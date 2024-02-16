#![no_std]
#![no_main]

// arduino and atmega hal
use arduino_hal::port::mode;
use panic_halt as _;

// stirage and heapless
use core::{borrow::Borrow, cell};
use heapless::Vec;

static TIME_SINCE_NEW_CYCLE: avr_device::interrupt::Mutex<cell::Cell<i32>> =
    avr_device::interrupt::Mutex::new(cell::Cell::new(0));

struct LocalPin {
    value: u8,
    port: arduino_hal::port::Pin<mode::Output>
}

impl ServoController{
    pub fn ConvertValueToCyclePeriod(i: u8) -> u8 {
        i
    }
    pub fn ToggleAllStates(self) {
        for mut i in self.pinPortVector {
            if 
                avr_device::interrupt::free(|cs| {TIME_SINCE_NEW_CYCLE.borrow(cs).get()})
            <= Self::ConvertValueToCyclePeriod(i.value) as i32 {
                i.port.set_high();
            } else {
                i.port.set_low();
            }
        }
    }
}

struct ServoController{
    pinPortVector: Vec<LocalPin, 10>,
    timeSinceLastTick: u8,
}

const fn GenSC() -> ServoController{
    let mut tvec: Vec<LocalPin, 10> = Vec::new();
    let mut tempsc = ServoController {
        pinPortVector: tvec,
        timeSinceLastTick: 0
    };
    tempsc
}

// storage - values and types
const PRESCALER: u32 = 8;
const TIMER_COUNTS: u32 = 125;

const MILLIS_INCREMENT: u32 = PRESCALER * TIMER_COUNTS / 16000;

static SERVO_CONTROLLER_CONTAINER: avr_device::interrupt::Mutex<cell::Cell<ServoController>> =
    avr_device::interrupt::Mutex::new(cell::Cell::new(GenSC()));



// timer code - timer will initialize the servocontroller et al
fn millis_init(tc0: arduino_hal::pac::TC0) {
    // Configure the timer for the above interval (in CTC mode)
    // and enable its interrupt.
    tc0.tccr0a.write(|w| w.wgm0().ctc());
    tc0.ocr0a.write(|w| w.bits(TIMER_COUNTS as u8));
    tc0.tccr0b.write(|w| match PRESCALER {
        8 => w.cs0().prescale_8(),
        64 => w.cs0().prescale_64(),
        256 => w.cs0().prescale_256(),
        1024 => w.cs0().prescale_1024(),
        _ => panic!(),
    });
    tc0.timsk0.write(|w| w.ocie0a().set_bit());

    /* Reset the global millisecond counter
    avr_device::interrupt::free(|cs| {
        MILLIS_COUNTER.borrow(cs).set(0);
    });*/
}

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    /*
     * For examples (and inspiration), head to
     *
     *     https://github.com/Rahix/avr-hal/tree/main/examples
     *
     * NOTE: Not all examples were ported to all boards!  There is a good chance though, that code
     * for a different board can be adapted for yours.  The Arduino Uno currently has the most
     * examples available.
     */

    let mut led = pins.d13.into_output();

    loop {
        led.toggle();
        arduino_hal::delay_ms(1000);
    }
}
