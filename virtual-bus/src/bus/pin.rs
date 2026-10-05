//! Pin adapters.
//!
//! [`InvertedPin`] inverts the logic of any [`OutputPin`] / [`InputPin`].
//! embedded-hal-bus's `ExclusiveDevice` and most drivers assume "drive CS low to select",
//! so for an IC with an active-high CS, wrap the CS pin in this before passing it on.
//! The same as putting an inverter in front of the pin on real hardware.

use embedded_hal::digital::{ErrorType, InputPin, OutputPin, StatefulOutputPin};

/// A pin with inverted logic. `set_low` drives the inner pin high; `is_high` is true when the inner pin is low
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InvertedPin<P> {
    inner: P,
}

impl<P> InvertedPin<P> {
    pub fn new(inner: P) -> Self {
        Self { inner }
    }

    pub fn inner(&self) -> &P {
        &self.inner
    }

    pub fn inner_mut(&mut self) -> &mut P {
        &mut self.inner
    }

    pub fn into_inner(self) -> P {
        self.inner
    }
}

impl<P: ErrorType> ErrorType for InvertedPin<P> {
    type Error = P::Error;
}

impl<P: OutputPin> OutputPin for InvertedPin<P> {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.inner.set_high()
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.inner.set_low()
    }
}

impl<P: StatefulOutputPin> StatefulOutputPin for InvertedPin<P> {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        self.inner.is_set_low()
    }

    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        self.inner.is_set_high()
    }
}

impl<P: InputPin> InputPin for InvertedPin<P> {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        self.inner.is_low()
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        self.inner.is_high()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::convert::Infallible;

    /// A pin that only remembers its level
    #[derive(Default)]
    struct Level(bool);

    impl ErrorType for Level {
        type Error = Infallible;
    }
    impl OutputPin for Level {
        fn set_low(&mut self) -> Result<(), Infallible> {
            self.0 = false;
            Ok(())
        }
        fn set_high(&mut self) -> Result<(), Infallible> {
            self.0 = true;
            Ok(())
        }
    }
    impl StatefulOutputPin for Level {
        fn is_set_high(&mut self) -> Result<bool, Infallible> {
            Ok(self.0)
        }
        fn is_set_low(&mut self) -> Result<bool, Infallible> {
            Ok(!self.0)
        }
    }
    impl InputPin for Level {
        fn is_high(&mut self) -> Result<bool, Infallible> {
            Ok(self.0)
        }
        fn is_low(&mut self) -> Result<bool, Infallible> {
            Ok(!self.0)
        }
    }

    #[test]
    fn every_direction_is_inverted() {
        let mut p = InvertedPin::new(Level::default());
        p.set_low().unwrap();
        assert!(p.inner().0);
        assert!(p.is_set_low().unwrap());
        assert!(p.is_low().unwrap());
        p.set_high().unwrap();
        assert!(!p.inner().0);
        assert!(p.is_set_high().unwrap());
        assert!(p.is_high().unwrap());
        p.toggle().unwrap();
        assert!(p.into_inner().0);
    }
}
