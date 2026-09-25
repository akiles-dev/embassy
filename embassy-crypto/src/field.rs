//! Arithmetic modulo a curve's field prime `p`, for point decompression.
//!
//! Variable-time: only ever applied to public points.

/// Field element: little-endian 32-bit limbs, canonical (in `[0, p)`).
type Fe<const L: usize> = [u32; L];

fn from_be<const L: usize>(bytes: &[u8]) -> Fe<L> {
    debug_assert_eq!(bytes.len(), 4 * L);
    let mut out = [0u32; L];
    for (i, limb) in out.iter_mut().enumerate() {
        let end = bytes.len() - 4 * i;
        *limb = u32::from_be_bytes([bytes[end - 4], bytes[end - 3], bytes[end - 2], bytes[end - 1]]);
    }
    out
}

fn to_be<const L: usize>(a: &Fe<L>, out: &mut [u8]) {
    debug_assert_eq!(out.len(), 4 * L);
    let len = out.len();
    for (i, limb) in a.iter().enumerate() {
        out[len - 4 * (i + 1)..len - 4 * i].copy_from_slice(&limb.to_be_bytes());
    }
}

fn lt<const L: usize>(a: &Fe<L>, b: &Fe<L>) -> bool {
    for i in (0..L).rev() {
        if a[i] != b[i] {
            return a[i] < b[i];
        }
    }
    false
}

fn add<const L: usize>(a: &Fe<L>, b: &Fe<L>) -> (Fe<L>, bool) {
    let mut out = [0u32; L];
    let mut carry = 0u64;
    for i in 0..L {
        let s = u64::from(a[i]) + u64::from(b[i]) + carry;
        out[i] = s as u32;
        carry = s >> 32;
    }
    (out, carry != 0)
}

fn sub<const L: usize>(a: &Fe<L>, b: &Fe<L>) -> (Fe<L>, bool) {
    let mut out = [0u32; L];
    let mut borrow = 0u64;
    for i in 0..L {
        let d = u64::from(a[i]).wrapping_sub(u64::from(b[i])).wrapping_sub(borrow);
        out[i] = d as u32;
        borrow = (d >> 63) & 1;
    }
    (out, borrow != 0)
}

fn small<const L: usize>(v: u32) -> Fe<L> {
    let mut out = [0u32; L];
    out[0] = v;
    out
}

/// Montgomery arithmetic modulo an odd prime, with `R = 2^(32L)`.
struct Field<const L: usize> {
    p: Fe<L>,
    /// `-p^-1 mod 2^32`.
    m0: u32,
    /// `R^2 mod p`.
    r2: Fe<L>,
}

impl<const L: usize> Field<L> {
    fn new(p: Fe<L>) -> Self {
        // Newton iteration: each step doubles the number of correct low bits.
        let mut inv = 1u32;
        for _ in 0..5 {
            inv = inv.wrapping_mul(2u32.wrapping_sub(p[0].wrapping_mul(inv)));
        }
        let mut f = Self {
            p,
            m0: inv.wrapping_neg(),
            r2: small(1),
        };
        for _ in 0..64 * L {
            f.r2 = f.add(&f.r2, &f.r2);
        }
        f
    }

    fn add(&self, a: &Fe<L>, b: &Fe<L>) -> Fe<L> {
        let (s, carry) = add(a, b);
        if carry || !lt(&s, &self.p) { sub(&s, &self.p).0 } else { s }
    }

    fn sub(&self, a: &Fe<L>, b: &Fe<L>) -> Fe<L> {
        let (d, borrow) = sub(a, b);
        if borrow { add(&d, &self.p).0 } else { d }
    }

    /// `a * b / R mod p` (CIOS).
    fn mul(&self, a: &Fe<L>, b: &Fe<L>) -> Fe<L> {
        let p = &self.p;
        let mut t = [0u32; L];
        let mut hi = 0u32;
        for &ai in a {
            let mut c = 0u64;
            for j in 0..L {
                let s = u64::from(t[j]) + u64::from(ai) * u64::from(b[j]) + c;
                t[j] = s as u32;
                c = s >> 32;
            }
            let s = u64::from(hi) + c;
            hi = s as u32;
            let top = (s >> 32) as u32;

            let m = t[0].wrapping_mul(self.m0);
            let s = u64::from(t[0]) + u64::from(m) * u64::from(p[0]);
            let mut c = s >> 32;
            for j in 1..L {
                let s = u64::from(t[j]) + u64::from(m) * u64::from(p[j]) + c;
                t[j - 1] = s as u32;
                c = s >> 32;
            }
            let s = u64::from(hi) + c;
            t[L - 1] = s as u32;
            hi = top + (s >> 32) as u32;
        }
        if hi != 0 || !lt(&t, p) { sub(&t, p).0 } else { t }
    }

    fn enter(&self, a: &Fe<L>) -> Fe<L> {
        self.mul(a, &self.r2)
    }

    fn leave(&self, a: &Fe<L>) -> Fe<L> {
        self.mul(a, &small(1))
    }

    /// `a^e`, in the Montgomery domain.
    fn pow(&self, a: &Fe<L>, e: &Fe<L>) -> Fe<L> {
        let mut r = self.enter(&small(1));
        for i in (0..32 * L).rev() {
            r = self.mul(&r, &r);
            if (e[i / 32] >> (i % 32)) & 1 == 1 {
                r = self.mul(&r, a);
            }
        }
        r
    }
}

/// Recover `y` from `x` on `y^2 = x^3 - 3x + b` over `p`, picking the root
/// whose parity matches `odd`. All values are big-endian, `4L` bytes.
///
/// Requires `p = 3 mod 4`. Returns `false` if `x` is not the X coordinate of
/// a point on the curve.
pub(crate) fn decompress_y<const L: usize>(p: &[u8], b: &[u8], x: &[u8], odd: bool, y: &mut [u8]) -> bool {
    let p: Fe<L> = from_be(p);
    debug_assert_eq!(p[0] & 3, 3);
    let x: Fe<L> = from_be(x);
    if !lt(&x, &p) {
        return false;
    }

    let f = Field::new(p);
    let xm = f.enter(&x);
    let rhs = f.sub(&f.mul(&xm, &xm), &f.enter(&small(3)));
    let rhs = f.add(&f.mul(&rhs, &xm), &f.enter(&from_be(b)));

    // Since p = 3 mod 4, a square root of a square `r` is `r^((p+1)/4)`.
    let (e, _) = add(&p, &small(1));
    let mut exp = [0u32; L];
    for i in 0..L {
        exp[i] = (e[i] >> 2) | e.get(i + 1).map_or(0, |n| n << 30);
    }
    let ym = f.pow(&rhs, &exp);
    if f.mul(&ym, &ym) != rhs {
        return false;
    }

    let mut root = f.leave(&ym);
    if (root[0] & 1 == 1) != odd {
        if root == [0; L] {
            return false;
        }
        root = sub(&p, &root).0;
    }
    to_be(&root, y);
    true
}
