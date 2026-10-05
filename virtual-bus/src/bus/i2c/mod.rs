//! I2C.
//!
//! - Transaction level: [`VirtualI2cBus`] + [`I2cSlave`] (this file)
//! - Pin level: [`bitbang::BitBangI2c`] drives the signal lines of [`sim::SimI2cBus`]
//!
//! With [`VirtualI2cBus::attach_i2c`], both paths can share one bus.

use std::cell::RefCell;
use std::collections::{BTreeMap, VecDeque};
use std::rc::Rc;

use embedded_hal::i2c::{ErrorKind, ErrorType, I2c, NoAcknowledgeSource, Operation};

pub mod bitbang;
pub mod sim;

/// Direction of an I2C transfer
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    Write,
    Read,
}

/// The slave NACKed a data byte
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Nack;

/// Transaction-level I2C slave model.
///
/// One transaction makes the following calls, in order:
///
/// ```text
/// start(dir) → write(..) or read(..) zero or more times → (start(dir) again if the direction changes) → stop()
/// ```
///
/// `start` is called on every START and repeated START. `write` / `read` may be called
/// several times within one transfer, so remember "the first byte of the transfer"
/// in `start` (the pin-level path calls them one byte at a time).
pub trait I2cSlave {
    /// Our address was ACKed after a START / repeated START
    fn start(&mut self, _dir: Direction) {}
    /// Receives bytes from the master. Return `Err(Nack)` to NACK the last byte
    fn write(&mut self, data: &[u8]) -> Result<(), Nack>;
    /// Fills in the bytes to send to the master
    fn read(&mut self, buf: &mut [u8]);
    /// STOP
    fn stop(&mut self) {}
}

impl<T: I2cSlave + ?Sized> I2cSlave for Rc<RefCell<T>> {
    fn start(&mut self, dir: Direction) {
        self.borrow_mut().start(dir)
    }
    fn write(&mut self, data: &[u8]) -> Result<(), Nack> {
        self.borrow_mut().write(data)
    }
    fn read(&mut self, buf: &mut [u8]) {
        self.borrow_mut().read(buf)
    }
    fn stop(&mut self) {
        self.borrow_mut().stop()
    }
}

impl<T: I2cSlave + ?Sized> I2cSlave for Box<T> {
    fn start(&mut self, dir: Direction) {
        (**self).start(dir)
    }
    fn write(&mut self, data: &[u8]) -> Result<(), Nack> {
        (**self).write(data)
    }
    fn read(&mut self, buf: &mut [u8]) {
        (**self).read(buf)
    }
    fn stop(&mut self) {
        (**self).stop()
    }
}

/// A fault injected with [`VirtualI2cBus::inject_fault`]. Affects only the next transaction
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Fault {
    /// NACK the address (the device does not respond)
    AddressNack,
    /// NACK the `index`-th byte of written data (0-based, counted over the whole transaction)
    DataNack { index: usize },
    /// Bus error (misplaced START / STOP and the like)
    BusError,
    /// Arbitration lost
    ArbitrationLoss,
}

/// One operation in the log
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum LogOp {
    Write(Vec<u8>),
    Read(Vec<u8>),
}

/// One transaction in the log
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LogEntry {
    pub address: u8,
    pub ops: Vec<LogOp>,
    pub result: Result<(), ErrorKind>,
}

/// Why a device could not be attached
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AttachError {
    /// Another device is already attached at this address
    AddressInUse(u8),
    /// The address does not fit in 7 bits
    InvalidAddress(u8),
}

/// Object-safe trait for forwarding to another `I2c` implementation
trait ForwardI2c {
    fn forward(&mut self, address: u8, ops: &mut [Operation<'_>]) -> Result<(), ErrorKind>;
}

impl<T: I2c> ForwardI2c for T {
    fn forward(&mut self, address: u8, ops: &mut [Operation<'_>]) -> Result<(), ErrorKind> {
        use embedded_hal::i2c::Error as _;
        self.transaction(address, ops).map_err(|e| e.kind())
    }
}

enum Device {
    Model(Box<dyn I2cSlave>),
    Forward(Box<dyn ForwardI2c>),
}

/// Transaction-level virtual I2C bus. Implements `embedded_hal::i2c::I2c`.
///
/// Routes each transaction to the model at its address. Following the embedded-hal rules, consecutive
/// operations in the same direction are merged into one transfer (no repeated START).
#[derive(Default)]
pub struct VirtualI2cBus {
    devices: BTreeMap<u8, Device>,
    faults: VecDeque<Fault>,
    log: Vec<LogEntry>,
}

impl VirtualI2cBus {
    pub fn new() -> Self {
        Self::default()
    }

    fn check_free(&self, address: u8) -> Result<(), AttachError> {
        if address > 0x7F {
            return Err(AttachError::InvalidAddress(address));
        }
        if self.devices.contains_key(&address) {
            return Err(AttachError::AddressInUse(address));
        }
        Ok(())
    }

    /// Attaches a transaction-level model.
    /// To look inside it later, wrap it with [`shared`](crate::shared) and pass a clone
    pub fn attach(
        &mut self,
        address: u8,
        slave: impl I2cSlave + 'static,
    ) -> Result<(), AttachError> {
        self.check_free(address)?;
        self.devices.insert(address, Device::Model(Box::new(slave)));
        Ok(())
    }

    /// Attaches another `I2c` implementation (such as a [`bitbang::BitBangI2c`] driving RTL)
    /// at `address`. Transactions to this address are forwarded as they are
    pub fn attach_i2c(&mut self, address: u8, i2c: impl I2c + 'static) -> Result<(), AttachError> {
        self.check_free(address)?;
        self.devices.insert(address, Device::Forward(Box::new(i2c)));
        Ok(())
    }

    /// Detaches a device. Returns true if one was attached
    pub fn detach(&mut self, address: u8) -> bool {
        self.devices.remove(&address).is_some()
    }

    /// The addresses with a device attached
    pub fn addresses(&self) -> Vec<u8> {
        self.devices.keys().copied().collect()
    }

    /// Injects a fault. Faults take effect one per transaction, starting with the next one, in the order injected
    pub fn inject_fault(&mut self, fault: Fault) {
        self.faults.push_back(fault);
    }

    /// The log of transactions so far
    pub fn log(&self) -> &[LogEntry] {
        &self.log
    }

    pub fn clear_log(&mut self) {
        self.log.clear();
    }

    fn run(
        device: Option<&mut Device>,
        address: u8,
        ops: &mut [Operation<'_>],
        fault: Option<Fault>,
    ) -> Result<(), ErrorKind> {
        match fault {
            Some(Fault::AddressNack) => {
                return Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address));
            }
            Some(Fault::BusError) => return Err(ErrorKind::Bus),
            Some(Fault::ArbitrationLoss) => return Err(ErrorKind::ArbitrationLoss),
            _ => {}
        }
        let device = device.ok_or(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address))?;
        let slave = match device {
            Device::Forward(f) => return f.forward(address, ops),
            Device::Model(m) => m,
        };
        let nack_at = match fault {
            Some(Fault::DataNack { index }) => Some(index),
            _ => None,
        };

        let mut written = 0usize;
        let mut i = 0;
        while i < ops.len() {
            let dir = direction(&ops[i]);
            let mut j = i;
            while j < ops.len() && direction(&ops[j]) == dir {
                j += 1;
            }
            slave.start(dir);
            match dir {
                Direction::Write => {
                    let data: Vec<u8> = ops[i..j]
                        .iter()
                        .flat_map(|op| match op {
                            Operation::Write(d) => d.to_vec(),
                            Operation::Read(_) => unreachable!(),
                        })
                        .collect();
                    if let Some(n) = nack_at.filter(|&n| n < written + data.len()) {
                        // accept up to the n-th byte, and NACK the n-th byte
                        let _ = slave.write(&data[..n - written]);
                        slave.stop();
                        return Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Data));
                    }
                    if slave.write(&data).is_err() {
                        slave.stop();
                        return Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Data));
                    }
                    written += data.len();
                }
                Direction::Read => {
                    let total: usize = ops[i..j]
                        .iter()
                        .map(|op| match op {
                            Operation::Read(b) => b.len(),
                            Operation::Write(_) => unreachable!(),
                        })
                        .sum();
                    let mut buf = vec![0u8; total];
                    slave.read(&mut buf);
                    let mut pos = 0;
                    for op in &mut ops[i..j] {
                        if let Operation::Read(b) = op {
                            b.copy_from_slice(&buf[pos..pos + b.len()]);
                            pos += b.len();
                        }
                    }
                }
            }
            i = j;
        }
        slave.stop();
        Ok(())
    }
}

fn direction(op: &Operation<'_>) -> Direction {
    match op {
        Operation::Write(_) => Direction::Write,
        Operation::Read(_) => Direction::Read,
    }
}

impl ErrorType for VirtualI2cBus {
    type Error = ErrorKind;
}

impl I2c for VirtualI2cBus {
    fn transaction(&mut self, address: u8, ops: &mut [Operation<'_>]) -> Result<(), ErrorKind> {
        let fault = self.faults.pop_front();
        let result = Self::run(self.devices.get_mut(&address), address, ops, fault);
        let ops = ops
            .iter()
            .map(|op| match op {
                Operation::Write(d) => LogOp::Write(d.to_vec()),
                Operation::Read(b) => LogOp::Read(b.to_vec()),
            })
            .collect();
        self.log.push(LogEntry {
            address,
            ops,
            result,
        });
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shared;

    /// A model that remembers everything it receives
    #[derive(Default)]
    struct Recorder {
        events: Vec<String>,
        next_read: u8,
        nack_after: Option<usize>,
        received: usize,
    }

    impl I2cSlave for Recorder {
        fn start(&mut self, dir: Direction) {
            self.events.push(format!("start {dir:?}"));
        }
        fn write(&mut self, data: &[u8]) -> Result<(), Nack> {
            self.events.push(format!("write {data:02x?}"));
            self.received += data.len();
            match self.nack_after {
                Some(n) if self.received > n => Err(Nack),
                _ => Ok(()),
            }
        }
        fn read(&mut self, buf: &mut [u8]) {
            for b in buf.iter_mut() {
                *b = self.next_read;
                self.next_read = self.next_read.wrapping_add(1);
            }
            self.events.push(format!("read {}", buf.len()));
        }
        fn stop(&mut self) {
            self.events.push("stop".into());
        }
    }

    #[test]
    fn routes_by_address_and_nacks_missing_device() {
        let a = shared(Recorder::default());
        let mut bus = VirtualI2cBus::new();
        bus.attach(0x20, a.clone()).unwrap();
        bus.write(0x20, &[1, 2]).unwrap();
        assert_eq!(
            bus.write(0x21, &[1]),
            Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address))
        );
        assert_eq!(a.borrow().events, ["start Write", "write [01, 02]", "stop"]);
        assert_eq!(bus.log().len(), 2);
        assert_eq!(bus.log()[1].address, 0x21);
    }

    #[test]
    fn merges_same_direction_operations() {
        let a = shared(Recorder::default());
        let mut bus = VirtualI2cBus::new();
        bus.attach(0x50, a.clone()).unwrap();
        let mut r1 = [0u8; 2];
        let mut r2 = [0u8; 1];
        bus.transaction(
            0x50,
            &mut [
                Operation::Write(&[0x10]),
                Operation::Write(&[0x11, 0x12]),
                Operation::Read(&mut r1),
                Operation::Read(&mut r2),
            ],
        )
        .unwrap();
        assert_eq!(
            a.borrow().events,
            [
                "start Write",
                "write [10, 11, 12]",
                "start Read",
                "read 3",
                "stop"
            ]
        );
        assert_eq!(r1, [0, 1]);
        assert_eq!(r2, [2]);
    }

    #[test]
    fn address_collision_is_rejected() {
        let mut bus = VirtualI2cBus::new();
        bus.attach(0x20, Recorder::default()).unwrap();
        assert_eq!(
            bus.attach(0x20, Recorder::default()),
            Err(AttachError::AddressInUse(0x20))
        );
        assert_eq!(
            bus.attach(0x80, Recorder::default()),
            Err(AttachError::InvalidAddress(0x80))
        );
        assert!(bus.detach(0x20));
        bus.attach(0x20, Recorder::default()).unwrap();
        assert_eq!(bus.addresses(), [0x20]);
    }

    #[test]
    fn injected_faults_apply_once_in_order() {
        let a = shared(Recorder::default());
        let mut bus = VirtualI2cBus::new();
        bus.attach(0x20, a.clone()).unwrap();
        bus.inject_fault(Fault::AddressNack);
        bus.inject_fault(Fault::BusError);
        bus.inject_fault(Fault::ArbitrationLoss);
        assert_eq!(
            bus.write(0x20, &[0]),
            Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address))
        );
        assert_eq!(bus.write(0x20, &[0]), Err(ErrorKind::Bus));
        assert_eq!(bus.write(0x20, &[0]), Err(ErrorKind::ArbitrationLoss));
        assert_eq!(bus.write(0x20, &[0]), Ok(()));
        // the 3 failed ones never reach the model
        assert_eq!(a.borrow().events, ["start Write", "write [00]", "stop"]);
    }

    #[test]
    fn data_nack_delivers_bytes_before_the_nacked_one() {
        let a = shared(Recorder::default());
        let mut bus = VirtualI2cBus::new();
        bus.attach(0x20, a.clone()).unwrap();
        bus.inject_fault(Fault::DataNack { index: 2 });
        assert_eq!(
            bus.write(0x20, &[0xA, 0xB, 0xC, 0xD]),
            Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Data))
        );
        assert_eq!(a.borrow().events, ["start Write", "write [0a, 0b]", "stop"]);
    }

    #[test]
    fn model_can_nack_data() {
        let a = shared(Recorder {
            nack_after: Some(1),
            ..Default::default()
        });
        let mut bus = VirtualI2cBus::new();
        bus.attach(0x20, a.clone()).unwrap();
        assert_eq!(
            bus.write(0x20, &[1, 2]),
            Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Data))
        );
        assert_eq!(
            bus.log()[0].result,
            Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Data))
        );
    }

    #[test]
    fn empty_write_probes_presence() {
        let mut bus = VirtualI2cBus::new();
        bus.attach(0x3C, Recorder::default()).unwrap();
        let found: Vec<u8> = (0x08..0x78)
            .filter(|&a| bus.write(a, &[]).is_ok())
            .collect();
        assert_eq!(found, [0x3C]);
    }

    #[test]
    fn attach_i2c_forwards_whole_transaction() {
        // use another VirtualI2cBus as the forwarding target
        let inner_model = shared(Recorder::default());
        let mut inner = VirtualI2cBus::new();
        inner.attach(0x29, inner_model.clone()).unwrap();

        let mut bus = VirtualI2cBus::new();
        bus.attach(0x20, Recorder::default()).unwrap();
        bus.attach_i2c(0x29, inner).unwrap();
        let mut buf = [0u8; 2];
        bus.write_read(0x29, &[0x0F], &mut buf).unwrap();
        assert_eq!(
            inner_model.borrow().events,
            ["start Write", "write [0f]", "start Read", "read 2", "stop"]
        );
    }
}
