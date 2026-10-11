//! Async buses for async drivers (Embassy and the like), and [`block_on`] to run them in a test.
//!
//! Works like a SystemVerilog testbench:
//!
//! - Every bus also implements its embedded-hal-async trait. Each async method calls the blocking
//!   one (same log, same faults) and finishes on the first poll
//! - The async `SimDelay` stops the task like `#delay`. [`block_on`] moves the clock when nothing
//!   else can run, so tasks run in time order
//! - [`SimPin`] is a wire to wait on (`digital::Wait`), like `wait` / `@(posedge)`
//! - Several tasks: `block_on(join(firmware, sensor))`, with `join` from embassy-futures, like
//!   `fork ... join`
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

use core::pin::{Pin, pin};
use core::task::{Context, Poll, Waker};
use std::cell::RefCell;
use std::convert::Infallible;
use std::future::poll_fn;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::task::Wake;

use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::{delay, i2c, spi};
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::I2c;
use embedded_hal_async::spi::{SpiBus, SpiDevice};

use crate::bus::i2c::VirtualI2cBus;
use crate::bus::i2c::bitbang::BitBangI2c;
use crate::bus::pin::{SimPin, Wire};
use crate::bus::spi::bitbang::BitBangSpi;
use crate::bus::spi::{SpiError, SpiSlave, VirtualSpiBus, VirtualSpiDevice};
use crate::bus::{SimClock, SimDelay};

/// Runs a future to the end and returns its value. The scheduler of the simulation.
///
/// - `Ready`: returns the value
/// - `Pending`, something woke it (`yield_now`, a [`SimPin`] change): polls again
/// - `Pending`, nothing woke it, a delay is waiting: moves that delay's clock to its deadline
///   (running the signal lines on it), wakes it, polls again
/// - `Pending`, nothing woke it, no delay waiting: panics. It waits for a real timer or an
///   interrupt that never comes
pub fn block_on<F: Future>(fut: F) -> F::Output {
    let woken = Arc::new(Woken(AtomicBool::new(false)));
    let waker = Waker::from(woken.clone());
    let mut cx = Context::from_waker(&waker);
    let mut fut = pin!(fut);
    loop {
        if let Poll::Ready(v) = fut.as_mut().poll(&mut cx) {
            return v;
        }
        if woken.0.swap(false, Ordering::Relaxed) {
            continue;
        }
        let Some(timer) = next_timer() else {
            panic!(
                "block_on: the future is waiting for something outside the virtual bus \
                 (a real timer, an interrupt, another task) and nothing here will wake it"
            );
        };
        let now = timer.clock.now_ps();
        if timer.deadline_ps > now {
            timer.clock.advance_ps(timer.deadline_ps - now);
        }
        timer.waker.wake();
    }
}

/// A task waiting for a clock to reach a time
struct Timer {
    id: u64,
    clock: SimClock,
    deadline_ps: u64,
    waker: Waker,
}

thread_local! {
    /// (last id, timers) of the delays waiting on this thread
    static TIMERS: RefCell<(u64, Vec<Timer>)> = const { RefCell::new((0, Vec::new())) };
}

/// Takes out the timer due first (the earliest registered on a tie)
fn next_timer() -> Option<Timer> {
    TIMERS.with_borrow_mut(|(_, timers)| {
        let i = (0..timers.len()).min_by_key(|&i| {
            timers[i]
                .deadline_ps
                .saturating_sub(timers[i].clock.now_ps())
        })?;
        Some(timers.remove(i))
    })
}

/// `#delay`: waits until the clock reaches the deadline. Leaves the timers when done or dropped
struct Sleep {
    clock: SimClock,
    deadline_ps: u64,
    id: Option<u64>,
}

impl Sleep {
    fn cancel(&mut self) {
        if let Some(id) = self.id.take() {
            TIMERS.with_borrow_mut(|(_, timers)| timers.retain(|t| t.id != id));
        }
    }
}

impl Future for Sleep {
    type Output = ();

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<()> {
        let this = self.get_mut();
        if this.clock.now_ps() >= this.deadline_ps {
            this.cancel();
            return Poll::Ready(());
        }
        let id = this.id;
        let new_id = TIMERS.with_borrow_mut(|(last, timers)| {
            if let Some(t) = id.and_then(|id| timers.iter_mut().find(|t| t.id == id)) {
                t.waker.clone_from(cx.waker());
                return id;
            }
            *last += 1;
            timers.push(Timer {
                id: *last,
                clock: this.clock.clone(),
                deadline_ps: this.deadline_ps,
                waker: cx.waker().clone(),
            });
            Some(*last)
        });
        this.id = new_id;
        Poll::Pending
    }
}

impl Drop for Sleep {
    fn drop(&mut self) {
        self.cancel();
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
    /// Stops the task until the clock has moved `ns` on. `block_on` moves it when nothing can run
    async fn delay_ns(&mut self, ns: u32) {
        Sleep {
            clock: self.clock.clone(),
            deadline_ps: self.clock.now_ps() + u64::from(ns) * 1000,
            id: None,
        }
        .await;
    }
}

async fn wait_until(pin: &SimPin, done: impl Fn(&Wire) -> bool) {
    poll_fn(|cx| {
        let mut w = pin.0.borrow_mut();
        if done(&w) {
            return Poll::Ready(());
        }
        w.waiting.push(cx.waker().clone());
        Poll::Pending
    })
    .await;
}

/// Edge waits count only the edges after the call, as on a real HAL
impl embedded_hal_async::digital::Wait for SimPin {
    async fn wait_for_high(&mut self) -> Result<(), Infallible> {
        wait_until(self, |w| w.high).await;
        Ok(())
    }

    async fn wait_for_low(&mut self) -> Result<(), Infallible> {
        wait_until(self, |w| !w.high).await;
        Ok(())
    }

    async fn wait_for_rising_edge(&mut self) -> Result<(), Infallible> {
        let start = self.0.borrow().rising;
        wait_until(self, |w| w.rising > start).await;
        Ok(())
    }

    async fn wait_for_falling_edge(&mut self) -> Result<(), Infallible> {
        let start = self.0.borrow().falling;
        wait_until(self, |w| w.falling > start).await;
        Ok(())
    }

    async fn wait_for_any_edge(&mut self) -> Result<(), Infallible> {
        let start = {
            let w = self.0.borrow();
            w.rising + w.falling
        };
        wait_until(self, |w| w.rising + w.falling > start).await;
        Ok(())
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

        // an async delay stops the task; block_on moves the clock
        let mut delay = lines.delay();
        let before = lines.now_ns();
        block_on(DelayNs::delay_us(&mut delay, 5));
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
