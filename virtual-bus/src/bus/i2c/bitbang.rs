//! Bit-bang I2C master on top of [`OutputPin`] / [`InputPin`] / [`DelayNs`].
//!
//! It implements [`I2c`] itself, so drivers see an ordinary I2C bus.
//! Pins are treated as open-drain (`set_low` pulls the line low, `set_high` releases it).
//! Works the same on real GPIOs and on the simulated [`crate::bus::i2c::sim::SimI2cBus`].
//!
//! Each bit consists of four phases (a quarter period each).
//!
//! ```text
//! SCL  ‾‾\__________/‾‾‾‾‾‾‾‾‾‾\__
//! SDA  ====X(change)============X==   SDA changes a quarter period after SCL falls
//!          |  1/4  |  1/4 | 1/4 | 1/4 |
//! ```
//!
//! Clock stretching, arbitration, multi-master and 10-bit addresses are not supported.

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::i2c::{ErrorKind, ErrorType, I2c, NoAcknowledgeSource, Operation};

pub struct BitBangI2c<SCL, SDA, D> {
    scl: SCL,
    sda: SDA,
    delay: D,
    quarter_ns: u32,
}

impl<SCL, SDA, D> BitBangI2c<SCL, SDA, D>
where
    SCL: OutputPin + InputPin,
    SDA: OutputPin + InputPin,
    D: DelayNs,
{
    /// `freq_hz` is the SCL frequency
    pub fn new(scl: SCL, sda: SDA, delay: D, freq_hz: u32) -> Self {
        assert!(freq_hz > 0);
        Self {
            scl,
            sda,
            delay,
            quarter_ns: (1_000_000_000 / freq_hz / 4).max(1),
        }
    }

    /// Returns the pins and the delay
    pub fn release(self) -> (SCL, SDA, D) {
        (self.scl, self.sda, self.delay)
    }

    fn q(&mut self) {
        self.delay.delay_ns(self.quarter_ns);
    }

    fn scl(&mut self, high: bool) -> Result<(), ErrorKind> {
        if high {
            self.scl.set_high()
        } else {
            self.scl.set_low()
        }
        .map_err(|_| ErrorKind::Other)
    }

    fn sda(&mut self, high: bool) -> Result<(), ErrorKind> {
        if high {
            self.sda.set_high()
        } else {
            self.sda.set_low()
        }
        .map_err(|_| ErrorKind::Other)
    }

    fn sda_is_high(&mut self) -> Result<bool, ErrorKind> {
        self.sda.is_high().map_err(|_| ErrorKind::Other)
    }

    /// START / repeated START. Before the call, the bus is either idle or SCL has been low for a quarter period
    fn start(&mut self) -> Result<(), ErrorKind> {
        self.sda(true)?;
        self.q();
        self.scl(true)?;
        self.q();
        self.q();
        if !self.sda_is_high()? {
            // someone is holding SDA low
            return Err(ErrorKind::Bus);
        }
        self.sda(false)?;
        self.q();
        self.q();
        self.scl(false)?;
        self.q();
        Ok(())
    }

    fn stop(&mut self) -> Result<(), ErrorKind> {
        self.sda(false)?;
        self.q();
        self.scl(true)?;
        self.q();
        self.q();
        self.sda(true)?;
        self.q();
        self.q();
        Ok(())
    }

    fn write_bit(&mut self, bit: bool) -> Result<(), ErrorKind> {
        self.sda(bit)?;
        self.q();
        self.scl(true)?;
        self.q();
        self.q();
        self.scl(false)?;
        self.q();
        Ok(())
    }

    fn read_bit(&mut self) -> Result<bool, ErrorKind> {
        self.sda(true)?;
        self.q();
        self.scl(true)?;
        self.q();
        let bit = self.sda_is_high()?;
        self.q();
        self.scl(false)?;
        self.q();
        Ok(bit)
    }

    /// Sends one byte and receives the ACK. Returns true on ACK
    fn write_byte(&mut self, byte: u8) -> Result<bool, ErrorKind> {
        for i in (0..8).rev() {
            self.write_bit(byte & (1 << i) != 0)?;
        }
        Ok(!self.read_bit()?)
    }

    fn read_byte(&mut self, ack: bool) -> Result<u8, ErrorKind> {
        let mut b = 0u8;
        for _ in 0..8 {
            b = (b << 1) | u8::from(self.read_bit()?);
        }
        self.write_bit(!ack)?;
        Ok(b)
    }

    fn run(&mut self, address: u8, ops: &mut [Operation<'_>]) -> Result<(), ErrorKind> {
        let mut prev_read: Option<bool> = None;
        for i in 0..ops.len() {
            let is_read = matches!(ops[i], Operation::Read(_));
            if prev_read != Some(is_read) {
                self.start()?;
                if !self.write_byte((address << 1) | u8::from(is_read))? {
                    return Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address));
                }
            }
            prev_read = Some(is_read);
            // if the next operation is also a read, ACK the last byte of this one too
            let next_is_read = matches!(ops.get(i + 1), Some(Operation::Read(_)));
            match &mut ops[i] {
                Operation::Write(data) => {
                    for &b in data.iter() {
                        if !self.write_byte(b)? {
                            return Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Data));
                        }
                    }
                }
                Operation::Read(buf) => {
                    let n = buf.len();
                    for (j, b) in buf.iter_mut().enumerate() {
                        let last = j + 1 == n && !next_is_read;
                        *b = self.read_byte(!last)?;
                    }
                }
            }
        }
        Ok(())
    }
}

impl<SCL, SDA, D> ErrorType for BitBangI2c<SCL, SDA, D> {
    type Error = ErrorKind;
}

impl<SCL, SDA, D> I2c for BitBangI2c<SCL, SDA, D>
where
    SCL: OutputPin + InputPin,
    SDA: OutputPin + InputPin,
    D: DelayNs,
{
    fn transaction(&mut self, address: u8, ops: &mut [Operation<'_>]) -> Result<(), ErrorKind> {
        if ops.is_empty() {
            return Ok(());
        }
        let result = self.run(address, ops);
        // release the bus with a STOP even on failure
        let stopped = self.stop();
        result.and(stopped)
    }
}
