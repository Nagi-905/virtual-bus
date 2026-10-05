//! Bit-bang SPI master on top of [`OutputPin`] / [`InputPin`] / [`DelayNs`].
//!
//! Implements [`SpiBus`] (no CS). Leave CS to `embedded_hal_bus::spi::ExclusiveDevice`,
//! or pass it to a driver that drives CS itself. Modes 0..=3, MSB first.
//!
//! MISO is read just before the capture edge. Each bit goes as follows (h = half period).
//!
//! ```text
//! CPHA = 0: set MOSI → h → read MISO → leading edge → h → trailing edge
//! CPHA = 1: leading edge → set MOSI → h → read MISO → trailing edge → h
//! ```

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::spi::{ErrorKind, ErrorType, Mode, Phase, Polarity, SpiBus};

pub struct BitBangSpi<SCK, MOSI, MISO, D> {
    sck: SCK,
    mosi: MOSI,
    miso: MISO,
    delay: D,
    mode: Mode,
    half_ns: u32,
}

impl<SCK, MOSI, MISO, D> BitBangSpi<SCK, MOSI, MISO, D>
where
    SCK: OutputPin,
    MOSI: OutputPin,
    MISO: InputPin,
    D: DelayNs,
{
    /// Puts SCK at its idle level and waits one period.
    ///
    /// The wait is there because if CS falls right after SCK goes idle, a slave that synchronizes
    /// SCK with a system clock would count "the change to the idle level" as an edge
    pub fn new(
        sck: SCK,
        mosi: MOSI,
        miso: MISO,
        delay: D,
        mode: Mode,
        freq_hz: u32,
    ) -> Result<Self, ErrorKind> {
        assert!(freq_hz > 0);
        let mut s = Self {
            sck,
            mosi,
            miso,
            delay,
            mode,
            half_ns: (1_000_000_000 / freq_hz / 2).max(1),
        };
        s.set_sck(false)?;
        s.mosi.set_low().map_err(|_| ErrorKind::Other)?;
        s.delay.delay_ns(2 * s.half_ns);
        Ok(s)
    }

    pub fn mode(&self) -> Mode {
        self.mode
    }

    /// Changes the mode, puts SCK at the new idle level and waits one period
    pub fn set_mode(&mut self, mode: Mode) -> Result<(), ErrorKind> {
        self.mode = mode;
        self.set_sck(false)?;
        self.delay.delay_ns(2 * self.half_ns);
        Ok(())
    }

    /// Returns the pins and the delay
    pub fn release(self) -> (SCK, MOSI, MISO, D) {
        (self.sck, self.mosi, self.miso, self.delay)
    }

    /// `active` = false is the idle level
    fn set_sck(&mut self, active: bool) -> Result<(), ErrorKind> {
        let idle_high = self.mode.polarity == Polarity::IdleHigh;
        if active != idle_high {
            self.sck.set_high()
        } else {
            self.sck.set_low()
        }
        .map_err(|_| ErrorKind::Other)
    }

    fn set_mosi(&mut self, bit: bool) -> Result<(), ErrorKind> {
        if bit {
            self.mosi.set_high()
        } else {
            self.mosi.set_low()
        }
        .map_err(|_| ErrorKind::Other)
    }

    fn read_miso(&mut self) -> Result<bool, ErrorKind> {
        self.miso.is_high().map_err(|_| ErrorKind::Other)
    }

    fn transfer_byte(&mut self, out: u8) -> Result<u8, ErrorKind> {
        let mut inb = 0u8;
        for i in (0..8).rev() {
            let bit = out & (1 << i) != 0;
            let sampled = match self.mode.phase {
                Phase::CaptureOnFirstTransition => {
                    self.set_mosi(bit)?;
                    self.delay.delay_ns(self.half_ns);
                    let s = self.read_miso()?;
                    self.set_sck(true)?;
                    self.delay.delay_ns(self.half_ns);
                    self.set_sck(false)?;
                    s
                }
                Phase::CaptureOnSecondTransition => {
                    self.set_sck(true)?;
                    self.set_mosi(bit)?;
                    self.delay.delay_ns(self.half_ns);
                    let s = self.read_miso()?;
                    self.set_sck(false)?;
                    self.delay.delay_ns(self.half_ns);
                    s
                }
            };
            inb = (inb << 1) | u8::from(sampled);
        }
        Ok(inb)
    }
}

impl<SCK, MOSI, MISO, D> ErrorType for BitBangSpi<SCK, MOSI, MISO, D> {
    type Error = ErrorKind;
}

impl<SCK, MOSI, MISO, D> SpiBus<u8> for BitBangSpi<SCK, MOSI, MISO, D>
where
    SCK: OutputPin,
    MOSI: OutputPin,
    MISO: InputPin,
    D: DelayNs,
{
    fn read(&mut self, words: &mut [u8]) -> Result<(), ErrorKind> {
        for w in words {
            *w = self.transfer_byte(0x00)?;
        }
        Ok(())
    }

    fn write(&mut self, words: &[u8]) -> Result<(), ErrorKind> {
        for &w in words {
            self.transfer_byte(w)?;
        }
        Ok(())
    }

    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), ErrorKind> {
        for i in 0..read.len().max(write.len()) {
            let miso = self.transfer_byte(write.get(i).copied().unwrap_or(0x00))?;
            if let Some(r) = read.get_mut(i) {
                *r = miso;
            }
        }
        Ok(())
    }

    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), ErrorKind> {
        for w in words {
            *w = self.transfer_byte(*w)?;
        }
        Ok(())
    }

    fn flush(&mut self) -> Result<(), ErrorKind> {
        Ok(())
    }
}
