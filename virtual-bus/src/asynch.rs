//! Async buses for async drivers (Embassy and the like), and [`block_on`] to run them in a test.
//!
//! - Every bus also implements its embedded-hal-async trait
//! - Each async method calls the blocking one: same log, same faults
//! - Nothing waits, so every future finishes on the first poll
//!
//! ```
//! use embedded_hal_async::i2c::I2c;
//! use virtual_bus::{VirtualI2cBus, block_on};
//! use virtual_bus::devices::register::{I2cFormat, RegisterMap};
//!
//! let regs = RegisterMap::new().ro(0x0F, 0xA5);
//! let mut bus = VirtualI2cBus::new();
//! bus.attach(0x29, regs.i2c(I2cFormat::new())).unwrap();
//!
//! let mut who = [0u8];
//! block_on(bus.write_read(0x29, &[0x0F], &mut who)).unwrap();
//! assert_eq!(who[0], 0xA5);
//! ```
//!
//! Several calls: `block_on(async { a.await; b.await })`.

use core::pin::pin;
use core::task::{Context, Poll, Waker};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::task::Wake;

use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::{delay, i2c, spi};
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::I2c;
use embedded_hal_async::spi::{SpiBus, SpiDevice};

use crate::bus::SimDelay;
use crate::bus::i2c::VirtualI2cBus;
use crate::bus::i2c::bitbang::BitBangI2c;
use crate::bus::spi::bitbang::BitBangSpi;
use crate::bus::spi::{SpiError, SpiSlave, VirtualSpiBus, VirtualSpiDevice};

/// Runs a future to the end and returns its value.
///
/// - `Ready`: returns the value
/// - `Pending`, woke itself (`yield_now`): polls again
/// - `Pending`, nothing woke it: panics. It waits for a timer or an interrupt that never comes
pub fn block_on<F: Future>(fut: F) -> F::Output {
    let woken = Arc::new(Woken(AtomicBool::new(false)));
    let waker = Waker::from(woken.clone());
    let mut cx = Context::from_waker(&waker);
    let mut fut = pin!(fut);
    loop {
        if let Poll::Ready(v) = fut.as_mut().poll(&mut cx) {
            return v;
        }
        if !woken.0.swap(false, Ordering::Relaxed) {
            panic!(
                "block_on: the future is waiting for something outside the virtual bus \
                 (a real timer, an interrupt, another task) and nothing here will wake it"
            );
        }
    }
}

/// Set when the future asks to be polled again
struct Woken(AtomicBool);

impl Wake for Woken {
    fn wake(self: Arc<Self>) {
        self.wake_by_ref();
    }

    fn wake_by_ref(self: &Arc<Self>) {
        self.0.store(true, Ordering::Relaxed);
    }
}

impl I2c for VirtualI2cBus {
    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        i2c::I2c::transaction(self, address, operations)
    }
}

impl<SCL, SDA, D> I2c for BitBangI2c<SCL, SDA, D>
where
    SCL: OutputPin + InputPin,
    SDA: OutputPin + InputPin,
    D: delay::DelayNs,
{
    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        i2c::I2c::transaction(self, address, operations)
    }
}

impl<S: SpiSlave> SpiDevice<u8> for VirtualSpiDevice<S> {
    async fn transaction(
        &mut self,
        operations: &mut [spi::Operation<'_, u8>],
    ) -> Result<(), SpiError> {
        spi::SpiDevice::transaction(self, operations)
    }
}

impl SpiBus<u8> for VirtualSpiBus {
    async fn read(&mut self, words: &mut [u8]) -> Result<(), SpiError> {
        spi::SpiBus::read(self, words)
    }

    async fn write(&mut self, words: &[u8]) -> Result<(), SpiError> {
        spi::SpiBus::write(self, words)
    }

    async fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), SpiError> {
        spi::SpiBus::transfer(self, read, write)
    }

    async fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), SpiError> {
        spi::SpiBus::transfer_in_place(self, words)
    }

    async fn flush(&mut self) -> Result<(), SpiError> {
        spi::SpiBus::flush(self)
    }
}

impl<SCK, MOSI, MISO, D> SpiBus<u8> for BitBangSpi<SCK, MOSI, MISO, D>
where
    SCK: OutputPin,
    MOSI: OutputPin,
    MISO: InputPin,
    D: delay::DelayNs,
{
    async fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        spi::SpiBus::read(self, words)
    }

    async fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        spi::SpiBus::write(self, words)
    }

    async fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        spi::SpiBus::transfer(self, read, write)
    }

    async fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        spi::SpiBus::transfer_in_place(self, words)
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        spi::SpiBus::flush(self)
    }
}

impl DelayNs for SimDelay {
    async fn delay_ns(&mut self, ns: u32) {
        delay::DelayNs::delay_ns(self, ns)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bus::i2c::sim::{PinLevelI2cSlave, SimI2cBus};
    use crate::bus::spi::sim::{PinLevelSpiSlave, SimSpiBus};
    use crate::bus::spi::virtual_spi_bus;
    use crate::devices::register::{I2cFormat, RegisterMap, SpiFormat};
    use crate::{Fault, Nack};
    use embedded_hal::i2c::{ErrorKind, NoAcknowledgeSource};
    use embedded_hal::spi::MODE_0;
    use embedded_hal_bus::spi::ExclusiveDevice;

    /// Polls once. Fails if the future is not done
    fn ready<F: Future>(fut: F) -> F::Output {
        match pin!(fut).poll(&mut Context::from_waker(Waker::noop())) {
            Poll::Ready(v) => v,
            Poll::Pending => panic!("the future did not finish on the first poll"),
        }
    }

    fn regs() -> RegisterMap {
        RegisterMap::new().ro(0x0F, 0xA5).rw(0x10, 0x00)
    }

    /// Like an async driver: generic over the async trait
    async fn read_whoami<I: I2c>(i2c: &mut I, address: u8) -> Result<u8, I::Error> {
        let mut b = [0u8];
        i2c.write_read(address, &[0x0F], &mut b).await?;
        Ok(b[0])
    }

    #[test]
    fn block_on_returns_the_output() {
        assert_eq!(block_on(async { 1 + 2 }), 3);
    }

    /// `Pending` n times, waking itself each time (like `yield_now`)
    struct YieldTimes(u32);

    impl Future for YieldTimes {
        type Output = ();
        fn poll(mut self: core::pin::Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<()> {
            if self.0 == 0 {
                return Poll::Ready(());
            }
            self.0 -= 1;
            cx.waker().wake_by_ref();
            Poll::Pending
        }
    }

    #[test]
    fn block_on_polls_again_a_future_that_woke_itself() {
        block_on(YieldTimes(3));
    }

    #[test]
    #[should_panic(expected = "nothing here will wake it")]
    fn block_on_panics_on_a_future_nothing_wakes() {
        block_on(core::future::pending::<()>());
    }

    #[test]
    #[should_panic(expected = "nothing here will wake it")]
    fn block_on_panics_when_a_yield_is_followed_by_a_real_wait() {
        block_on(async {
            YieldTimes(2).await;
            core::future::pending::<()>().await;
        });
    }

    #[test]
    fn ready_catches_a_waiting_future() {
        let caught = std::panic::catch_unwind(|| ready(core::future::pending::<()>()));
        assert!(caught.is_err());
    }

    #[test]
    fn virtual_i2c_bus() {
        let map = regs();
        let mut bus = VirtualI2cBus::new();
        bus.attach(0x29, map.i2c(I2cFormat::new())).unwrap();

        assert_eq!(ready(read_whoami(&mut bus, 0x29)), Ok(0xA5));
        ready(I2c::write(&mut bus, 0x29, &[0x10, 0x5A])).unwrap();
        assert_eq!(map.get(0x10), 0x5A);

        // same log and faults as the blocking calls
        assert_eq!(bus.log().len(), 2);
        bus.inject_fault(Fault::DataNack { index: 1 });
        assert_eq!(
            ready(I2c::write(&mut bus, 0x29, &[0x10, 0x00])),
            Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Data))
        );
        assert_eq!(
            ready(read_whoami(&mut bus, 0x30)),
            Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address))
        );
    }

    #[test]
    fn several_calls_in_one_block() {
        let map = regs();
        let mut bus = VirtualI2cBus::new();
        bus.attach(0x29, map.i2c(I2cFormat::new())).unwrap();
        let who = block_on(async {
            I2c::write(&mut bus, 0x29, &[0x10, 0x77]).await?;
            read_whoami(&mut bus, 0x29).await
        });
        assert_eq!(who, Ok(0xA5));
        assert_eq!(map.get(0x10), 0x77);
    }

    #[test]
    fn bitbang_i2c_on_the_signal_lines() {
        let lines = SimI2cBus::new();
        lines.attach(PinLevelI2cSlave::new(0x29, regs().i2c(I2cFormat::new())));
        let mut i2c = lines.master(400_000);
        let before = lines.now_ns();
        assert_eq!(ready(read_whoami(&mut i2c, 0x29)), Ok(0xA5));
        assert!(lines.now_ns() > before, "simulated time moved");
    }

    #[test]
    fn virtual_spi_device() {
        let map = regs();
        let mut dev = VirtualSpiDevice::new(map.spi(SpiFormat::read_bit7_inc_bit6()), MODE_0);
        let mut b = [0x8F, 0x00];
        ready(SpiDevice::transfer_in_place(&mut dev, &mut b)).unwrap();
        assert_eq!(b[1], 0xA5);
        ready(SpiDevice::write(&mut dev, &[0x10, 0x3C])).unwrap();
        assert_eq!(map.get(0x10), 0x3C);
    }

    #[test]
    fn virtual_spi_bus_with_async_exclusive_device() {
        let (bus, cs) = virtual_spi_bus(regs().spi(SpiFormat::read_bit7_inc_bit6()), MODE_0);
        // async ExclusiveDevice needs an async SpiBus and DelayNs
        let mut dev = ExclusiveDevice::new_no_delay(bus, cs).unwrap();
        let mut b = [0x8F, 0x00];
        ready(SpiDevice::transfer_in_place(&mut dev, &mut b)).unwrap();
        assert_eq!(b[1], 0xA5);
    }

    #[test]
    fn bitbang_spi_and_sim_delay() {
        let lines = SimSpiBus::new();
        let cs = lines.add_device(PinLevelSpiSlave::new(
            regs().spi(SpiFormat::read_bit7_inc_bit6()),
            MODE_0,
        ));
        let spi = lines.master(MODE_0, 1_000_000).unwrap();
        let mut dev = ExclusiveDevice::new(spi, cs, lines.delay()).unwrap();
        let mut b = [0x8F, 0x00];
        ready(SpiDevice::transfer_in_place(&mut dev, &mut b)).unwrap();
        assert_eq!(b[1], 0xA5);

        let mut delay = lines.delay();
        let before = lines.now_ns();
        ready(DelayNs::delay_us(&mut delay, 5));
        assert_eq!(lines.now_ns() - before, 5_000);
    }

    #[test]
    fn a_model_nack_reaches_the_async_caller() {
        struct Busy;
        impl crate::I2cSlave for Busy {
            fn start(&mut self, _: crate::Direction) -> Result<(), Nack> {
                Err(Nack)
            }
            fn write(&mut self, _: &[u8]) -> Result<(), Nack> {
                Ok(())
            }
            fn read(&mut self, _: &mut [u8]) {}
        }
        let mut bus = VirtualI2cBus::new();
        bus.attach(0x50, Busy).unwrap();
        assert_eq!(
            ready(read_whoami(&mut bus, 0x50)),
            Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address))
        );
    }
}
