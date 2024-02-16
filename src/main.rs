#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]
#![feature(trivial_bounds)]

// arduino and atmega hal
use arduino_hal::port::mode;
use panic_halt as _;

// stirage and heapless
use core::{borrow::Borrow, cell};
use heapless::Vec;

static TIME_SINCE_NEW_CYCLE: avr_device::interrupt::Mutex<cell::Cell<u32>> =
    avr_device::interrupt::Mutex::new(cell::Cell::new(0));

struct LocalPin {
    value: u32,
    port: arduino_hal::port::Pin<mode::Output>
}

impl ServoController{
    pub fn ConvertValueToCyclePeriod(i: u32) -> u32 {
        i // todo: make this the actual cycle value period conversion
    }
    pub fn ToggleAllStates(self) {
        for mut i in self.pinPortVector {
            if 
                avr_device::interrupt::free(|cs| {TIME_SINCE_NEW_CYCLE.borrow(cs).get()})
            <= Self::ConvertValueToCyclePeriod(i.value) {
                i.port.set_high();
            } else {
                i.port.set_low();
            }
        }
    }
    pub fn AddServoToArr(mut self, v: LocalPin) {
        self.pinPortVector.push(v);
    }
}

struct ServoController{
    pinPortVector: Vec<LocalPin, 10>,
    timeSinceLastTick: u8,
}

const fn gen_sc() -> ServoController{
    let mut tvec: Vec<LocalPin, 10> = Vec::new();
    let mut tempsc = ServoController {
        pinPortVector: tvec,
        timeSinceLastTick: 0
    };
    tempsc
}

// storage - values and types
const PRESCALER: u32 = 8;
const TIMER_COUNTS: u32 = 125; // each cycle occurs at 0.0625ms  past the last

const MILLIS_INCREMENT: u32 = PRESCALER * TIMER_COUNTS / 16000;

static SERVO_CONTROLLER_CONTAINER: avr_device::interrupt::Mutex<cell::Cell<ServoController>> =
    avr_device::interrupt::Mutex::new(cell::Cell::new(gen_sc()));

// timer code - timer will initialize the servocontroller et al
fn timer_init(tc0: arduino_hal::pac::TC0) {
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

    // Reset the global millisecond counter
    avr_device::interrupt::free(|cs| {
        TIME_SINCE_NEW_CYCLE.borrow(cs).set(0);
    });
}

#[avr_device::interrupt(atmega328p)]
fn TIMER0_COMPA() {
    avr_device::interrupt::free(|cs| {
        let counter_cell = TIME_SINCE_NEW_CYCLE.borrow(cs);
        let counter = counter_cell.get();
        counter_cell.set(counter + MILLIS_INCREMENT);
    })
}

fn attach(i: LocalPin)
where
ServoController: core::marker::Copy,
ServoController: Clone{
    avr_device::interrupt::free(|cs| {
        let counter_cell = SERVO_CONTROLLER_CONTAINER.borrow(cs);
        let counter = counter_cell.get();
        //counter_cell.set()
    })
}

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);
    let mut serial = arduino_hal::default_serial!(dp, pins, 57600);

    timer_init(dp.TC0);

    // Enable interrupts globally
    unsafe { avr_device::interrupt::enable() };

    let mut led = pins.d13.into_output();

    loop {
        led.toggle();
        arduino_hal::delay_ms(1000);
    }
}
