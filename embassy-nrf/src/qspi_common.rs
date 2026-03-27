//! Common types shared between the QSPI and sQSPI drivers.

/// QSPI bus frequency.
#[repr(u8)]
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Frequency {
    /// 32 Mhz
    M32 = 0,
    /// 16 Mhz
    M16 = 1,
    /// 10.7 Mhz
    M10_7 = 2,
    /// 8 Mhz
    M8 = 3,
    /// 6.4 Mhz
    M6_4 = 4,
    /// 5.3 Mhz
    M5_3 = 5,
    /// 4.6 Mhz
    M4_6 = 6,
    /// 4 Mhz
    M4 = 7,
    /// 3.6 Mhz
    M3_6 = 8,
    /// 3.2 Mhz
    M3_2 = 9,
    /// 2.9 Mhz
    M2_9 = 10,
    /// 2.7 Mhz
    M2_7 = 11,
    /// 2.5 Mhz
    M2_5 = 12,
    /// 2.3 Mhz
    M2_3 = 13,
    /// 2.1 Mhz
    M2_1 = 14,
    /// 2 Mhz
    M2 = 15,
}

/// SPI clock polarity.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Polarity {
    /// Clock idle low.
    IdleLow,
    /// Clock idle high.
    IdleHigh,
}

/// SPI clock phase.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Phase {
    /// Data captured on first clock edge.
    CaptureOnFirstTransition,
    /// Data captured on second clock edge.
    CaptureOnSecondTransition,
}

/// SPI mode (polarity + phase).
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Mode {
    /// Clock polarity.
    pub polarity: Polarity,
    /// Clock phase.
    pub phase: Phase,
}

/// SPI Mode 0: CPOL=0, CPHA=0.
pub const MODE_0: Mode = Mode {
    polarity: Polarity::IdleLow,
    phase: Phase::CaptureOnFirstTransition,
};

/// SPI Mode 1: CPOL=0, CPHA=1.
pub const MODE_1: Mode = Mode {
    polarity: Polarity::IdleLow,
    phase: Phase::CaptureOnSecondTransition,
};

/// SPI Mode 2: CPOL=1, CPHA=0.
pub const MODE_2: Mode = Mode {
    polarity: Polarity::IdleHigh,
    phase: Phase::CaptureOnFirstTransition,
};

/// SPI Mode 3: CPOL=1, CPHA=1.
pub const MODE_3: Mode = Mode {
    polarity: Polarity::IdleHigh,
    phase: Phase::CaptureOnSecondTransition,
};

/// Address mode (24-bit or 32-bit).
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AddressMode {
    /// 24-bit addressing (3 bytes).
    _24Bit,
    /// 32-bit addressing (4 bytes).
    _32Bit,
}
