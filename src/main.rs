#![deny(warnings)]
//#![deny(unsafe_code)]
#![no_main]
#![no_std]

 extern crate panic_halt;
// extern crate panic_itm;
// extern crate panic_semihosting;


use core::cell::RefCell;
use cortex_m::interrupt::{self, Mutex};

#[cfg(feature = "stm32h7x")]
use stm32h7xx_hal as p_hal;

#[cfg(feature = "stm32f4x")]
use stm32f4xx_hal as p_hal;

#[cfg(feature = "stm32f3x")]
use stm32f3xx_hal as p_hal;

use p_hal::prelude::*;
use p_hal::stm32 as pac;
use p_hal::stm32 as stm32;
use pac::I2C1;
//use pac::DWT;


#[macro_use]
extern crate cortex_m_rt;

use cortex_m_rt::{entry, ExceptionFrame};

use cmsis_rtos2;


#[cfg(debug_assertions)]
use cortex_m_log::{println};

use cortex_m_log::{d_println};

// FOR itm mode:
//#[cfg(debug_assertions)]
//use cortex_m_log::{
//  destination::Itm, printer::itm::InterruptSync as InterruptSyncItm,
//};

// #[cfg(debug_assertions)]
// use cortex_m_log::printer::semihosting;

//#[cfg(debug_assertions)]
//use cortex_m_semihosting;


use p_hal::gpio::GpioExt;
use p_hal::rcc::RccExt;

use core::ops::{DerefMut};

use core::ptr::{null, null_mut};
use bno080::wrapper::BNO080;
// use bno080::interface::{I2cInterface};
use bno080::interface::{SpiInterface};
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::ToggleableOutputPin;


const SYSCLOCK_TICK_RATE_HZ: u16 = 1000;
const HEARTBEAT_BLINK_RATE_HZ: u16 =  1;
const IMU_REPORTING_RATE_HZ: u16 = 100;
const IMU_REPORTING_INTERVAL_TICKS: u16 = (SYSCLOCK_TICK_RATE_HZ / IMU_REPORTING_RATE_HZ) ;
const IMU_REPORTING_INTERVAL_MS: u16 = (1000 / IMU_REPORTING_RATE_HZ) ;


#[cfg(feature = "stm32f3x")]
#[allow(non_upper_case_globals)]
#[no_mangle]
pub static SystemCoreClock: u32 =  8_000_000; //same as stm32f3xx_hal::rcc::HSI

#[cfg(feature = "stm32f4x")]
#[allow(non_upper_case_globals)]
#[no_mangle]
// pub static SystemCoreClock: u32 =  16_000_000; //same as stm32f4xx_hal::rcc::HSI
pub static SystemCoreClock: u32 =  25_000_000; // eg stm32f401 board with 25 MHz xtal HSE
// pub static SystemCoreClock: u32 =  8_000_000; // eg stm32f407 board with 8 MHz xtal HSE

#[cfg(feature = "stm32h7x")]
#[allow(non_upper_case_globals)]
#[no_mangle]
pub static SystemCoreClock: u32 = 48_000_000; //stm32h743 HSI (48 MHz)



#[cfg(feature = "stm32f3x")]
type ImuI2cPortType = p_hal::i2c::I2c<I2C1,
  (p_hal::gpio::gpiob::PB8<p_hal::gpio::AF4>,
   p_hal::gpio::gpiob::PB9<p_hal::gpio::AF4>)
>;

#[cfg(feature = "stm32f4x")]
pub type ImuI2cPortType = p_hal::i2c::I2c<I2C1,
 (p_hal::gpio::gpiob::PB8<p_hal::gpio::AlternateOD<p_hal::gpio::AF4>>,
  p_hal::gpio::gpiob::PB9<p_hal::gpio::AlternateOD<p_hal::gpio::AF4>>)
>;

#[cfg(feature = "stm32h7x")]
pub type ImuI2cPortType = p_hal::i2c::I2c<I2C1,
  (p_hal::gpio::gpiob::PB8<p_hal::gpio::Alternate<p_hal::gpio::AF4>>,
   p_hal::gpio::gpiob::PB9<p_hal::gpio::Alternate<p_hal::gpio::AF4>>)
>;
// type ImuDriverType = bno080::wrapper::BNO080<I2cInterface<ImuI2cPortType>>;

#[cfg(feature = "stm32f4x")]
pub type SpiPortType = p_hal::spi::Spi<stm32::SPI1,
  (
    p_hal::gpio::gpiob::PB3<p_hal::gpio::Alternate<p_hal::gpio::AF5>>, //SCLK
    p_hal::gpio::gpiob::PB4<p_hal::gpio::Alternate<p_hal::gpio::AF5>>, //MISO?
    p_hal::gpio::gpiob::PB5<p_hal::gpio::Alternate<p_hal::gpio::AF5>>, //MOSI?
  )
>;

type ChipSelectPinType = p_hal::gpio::gpiob::PB2<p_hal::gpio::Output<p_hal::gpio::PushPull>>; //CSN
type HIntPinType =  p_hal::gpio::gpiob::PB0<p_hal::gpio::Input<p_hal::gpio::Floating>>; //HINTN
type WakePinType =  p_hal::gpio::gpiob::PB1<p_hal::gpio::Output<p_hal::gpio::PushPull>>; // WAKE
type ImuDriverType = bno080::wrapper::BNO080<SpiInterface<SpiPortType, ChipSelectPinType, HIntPinType, WakePinType>>;



#[cfg(debug_assertions)]
type DebugLog = cortex_m_log::printer::dummy::Dummy;
//type DebugLog = cortex_m_log::printer::semihosting::Semihosting<cortex_m_log::modes::InterruptFree, cortex_m_semihosting::hio::HStdout>;


#[cfg(feature = "stm32f3x")]
type GpioTypeUserLed1 =  p_hal::gpio::gpiob::PB6<p_hal::gpio::Output<p_hal::gpio::PushPull>>;

#[cfg(feature = "stm32f4x")]
type GpioTypeUserLed1 =  p_hal::gpio::gpioc::PC13<p_hal::gpio::Output<p_hal::gpio::PushPull>>; //stm32f401CxUx
// type GpioTypeUserLed1 =  p_hal::gpio::gpiod::PD12<p_hal::gpio::Output<p_hal::gpio::PushPull>>; //stm32f407disco

#[cfg(feature = "stm32h7x")]
type GpioTypeUserLed1 =  p_hal::gpio::gpiob::PB0<p_hal::gpio::Output<p_hal::gpio::PushPull>>;


static USER_LED_1:  Mutex<RefCell<Option< GpioTypeUserLed1>>> = Mutex::new(RefCell::new(None));
static APP_DELAY_SOURCE: Mutex<RefCell<Option<  p_hal::delay::Delay >>> = Mutex::new(RefCell::new(None));
static IMU_DRIVER: Mutex<RefCell<Option< ImuDriverType>>> = Mutex::new(RefCell::new(None));

//static APP_ITM: Mutex<RefCell<Option< cortex_m::peripheral::ITM >>> = Mutex::new(RefCell::new(None));



// cortex-m-rt is setup to call DefaultHandler for a number of fault conditions
// we can override this in debug mode for handy debugging
#[exception]
fn DefaultHandler(_irqn: i16) {
  d_println!(get_debug_log(), "IRQn = {}", _irqn);
}


// cortex-m-rt calls this for serious faults.  can set a breakpoint to debug
#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
  panic!("HardFault: {:?}", ef);
}

#[exception]
unsafe fn MemoryManagement() -> ! {
  d_println!(get_debug_log(),  "MemoryManagement");
  loop { }
}

#[exception]
unsafe fn BusFault() -> ! {
  d_println!(get_debug_log(),  "BusFault");
  loop { }
}

#[exception]
unsafe fn UsageFault() -> ! {
  d_println!(get_debug_log(),  "UsageFault");
  loop { }
}


/// Called by rtos on assert failures
#[no_mangle]
extern "C" fn handle_assert_failed() -> ! {
  loop {
    cortex_m::asm::nop();
    cortex_m::asm::bkpt();
  }
}


/// Used in debug builds to provide a logging outlet
#[cfg(debug_assertions)]
fn get_debug_log() -> DebugLog {
  cortex_m_log::printer::Dummy::new()
  //semihosting::InterruptFree::<_>::stdout().unwrap()

}


// Toggle the user leds from their prior state
fn toggle_leds() {
  interrupt::free(|cs| {
    if let Some(ref mut led1) = USER_LED_1.borrow(cs).borrow_mut().deref_mut() {
      led1.toggle().unwrap();
    }
  });
}

pub fn setup_imu() {
  // initialize the IMU
  let mut res = Ok(());
  interrupt::free(|cs| {
    if let Some(ref mut imu_driver) = IMU_DRIVER.borrow(cs).borrow_mut().deref_mut() {
      if let Some(delay_source) = APP_DELAY_SOURCE.borrow(cs).borrow_mut().deref_mut() {
        res = imu_driver.init(delay_source);
        if res.is_ok() {
          res = imu_driver.enable_rotation_vector(IMU_REPORTING_INTERVAL_MS);
        }
      }
    }
  });

  if res.is_err() {
    d_println!(get_debug_log(), "bno080 init err {:?}", res);
  }
  else {
    d_println!(get_debug_log(), "bno080 OK");
    setup_imu_task();
  }
}




#[no_mangle]
extern "C" fn task_blink(_arg: *mut cty::c_void) {
  let tick_hz:u16 = cmsis_rtos2::rtos_kernel_get_tick_freq_hz() as u16;
  let tick_interval = (tick_hz / HEARTBEAT_BLINK_RATE_HZ) as u32;

  loop {
    toggle_leds();
    cmsis_rtos2::rtos_os_delay(tick_interval);
  }
}

pub fn setup_blink_task() {
  let tid = cmsis_rtos2::rtos_os_thread_new(
    Some(task_blink),
    null_mut(),
    null(),
  );
  if tid.is_null() {
    d_println!(get_debug_log(), "setup_blink_task failed!");
  }
}


/// Give some time to the IMU driver to process sensor reports
pub fn spin_imu_driver() -> ! {

  let tick_hz:u16 = cmsis_rtos2::rtos_kernel_get_tick_freq_hz() as u16;
  let tick_interval = (tick_hz / IMU_REPORTING_RATE_HZ ) as u32;
  if tick_interval != (IMU_REPORTING_INTERVAL_TICKS as u32) {
    d_println!(get_debug_log(), "unexpected tick rate {} vs {}", tick_interval, IMU_REPORTING_INTERVAL_TICKS);
  }

  loop {
    // let mut msg_count = 0;
    interrupt::free(|cs| {
      if let Some(ref mut imu_driver) = IMU_DRIVER.borrow(cs).borrow_mut().deref_mut() {
        //msg_count = imu_driver.get_received_packet_count();
        //msg_count = imu_driver.handle_all_messages();
        imu_driver.handle_one_message();
      }
    });

    cmsis_rtos2::rtos_os_delay(tick_interval);

  }

}


/// RTOS callback for starting the imu task
#[no_mangle]
extern "C" fn task_imu_driver(_arg: *mut cty::c_void) {
  spin_imu_driver();
}


pub fn setup_imu_task() {

 let tid = cmsis_rtos2::rtos_os_thread_new(
   Some(task_imu_driver),
   null_mut(),
   null(),
 );

 if tid.is_null() {
   d_println!(get_debug_log(), "setup_imu_task failed!");
 }
}


#[cfg(feature = "stm32f3x")]
fn setup_peripherals_f3x()  {
  let dp = pac::Peripherals::take().unwrap();
  let cp = cortex_m::Peripherals::take().unwrap();

  // Set up the system clock
  let mut rcc = dp.RCC.constrain();
  let mut flash = dp.FLASH.constrain();

  //use the existing sysclk
  let clocks = rcc.cfgr.freeze(&mut flash.acr);

  let delay_source =  p_hal::delay::Delay::new(cp.SYST, clocks);

  let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);
  //let mut gpiod = dp.GPIOD.split(&mut rcc.ahb);

  let mut user_led1 = gpiob.pb6.into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
  //set initial states of user LEDs
  user_led1.set_high().unwrap();

  // setup i2c1 and imu driver
  let scl = gpiob.pb8
      .into_open_drain_output(&mut gpiob.moder, &mut gpiob.otyper)
      .into_af4(&mut gpiob.moder, &mut gpiob.afrh);
  //scl.internal_pull_up(&mut gpiob.pupdr, true);

  let sda = gpiob.pb9
      .into_open_drain_output(&mut gpiob.moder, &mut gpiob.otyper)
      .into_af4(&mut gpiob.moder, &mut gpiob.afrh);
  //sda.internal_pull_up(&mut gpiob.pupdr, true);

  let imu_i2c_port = p_hal::i2c::I2c::i2c1(
    dp.I2C1, (scl, sda), 400.khz(), clocks, &mut rcc.apb1);
  let imu_driver = BNO080::new_with_interface(imu_i2c_port);

  //store shared peripherals
  interrupt::free(|cs| {
    USER_LED_1.borrow(cs).replace(Some(user_led1));
    IMU_DRIVER.borrow(cs).replace(Some(imu_driver));
    APP_DELAY_SOURCE.borrow(cs).replace(Some(delay_source));
  });

}

#[cfg(feature = "stm32f4x")]
fn setup_peripherals_f4x()  {
  let dp = pac::Peripherals::take().unwrap();
  let cp = cortex_m::Peripherals::take().unwrap();

  // Set up the system clock
  let rcc = dp.RCC.constrain();
  // HSI: use default internal oscillator
  //  let clocks = rcc.cfgr.freeze();
  // HSE: external crystal oscillator must be connected
  let clocks = rcc.cfgr.use_hse(SystemCoreClock.hz()).freeze();
  // let clocks = rcc.cfgr.sysclk(SystemCoreClock.hz()).freeze();
  // let clocks = rcc.cfgr.sysclk(48.mhz()).freeze();

  let delay_source =  p_hal::delay::Delay::new(cp.SYST, clocks);

  let gpiob = dp.GPIOB.split();
  let gpioc = dp.GPIOC.split();
  // let gpiod = dp.GPIOD.split();

  // let mut user_led1 = gpiod.pd12.into_push_pull_output(); //f4discovery
 let mut user_led1 = gpioc.pc13.into_push_pull_output(); //f401CxUx
  //set initial states of user LEDs
  user_led1.set_high().unwrap();

  //SPI1_SCK
  let sck = gpiob.pb3
      .into_push_pull_output()
      .into_alternate_af5();
  //SPI1_MISO
  let miso = gpiob.pb4
      .into_floating_input()
      .into_alternate_af5();
  //SPI1_MOSI
  let mosi = gpiob.pb5
      .into_push_pull_output()
      .into_alternate_af5();

  let spi_port = p_hal::spi::Spi::spi1(
    dp.SPI1, (sck, miso, mosi),
    embedded_hal::spi::MODE_0, 3_000_000.hz(), clocks );

  // SPI chip select CS
  let cs = gpiob.pb2
      .into_push_pull_output();

  // WAKEN pin
  let waken = gpiob.pb1
      .into_push_pull_output();

  // HINTN interrupt pin
  let hintn = gpiob.pb0
      .into_floating_input();

  let spi_iface = bno080::interface::SpiInterface::new(spi_port, cs, hintn, waken);
  let imu_driver = BNO080::new_with_interface(spi_iface);


  //  // setup i2c1 and imu driver
 //  // NOTE: eg f407 discovery board already has external pull-ups
 // let scl = gpiob.pb8
 //     .into_alternate_af4()
 //     .internal_pull_up(true)
 //     .set_open_drain();
 //
 //  let sda = gpiob.pb9
 //      .into_alternate_af4()
 //      .internal_pull_up(true)
 //      .set_open_drain();
 //  let imu_i2c_port = p_hal::i2c::I2c::i2c1(dp.I2C1, (scl, sda), 400.khz(), clocks);//TODO raise to 4oo khz
 //  let iface = I2cInterface::new(imu_i2c_port, bno080::interface::i2c::DEFAULT_ADDRESS);
 //  let imu_driver = BNO080::new_with_interface(iface);

  //store shared peripherals
  interrupt::free(|cs| {
    USER_LED_1.borrow(cs).replace(Some(user_led1));
    IMU_DRIVER.borrow(cs).replace(Some(imu_driver));
    APP_DELAY_SOURCE.borrow(cs).replace(Some(delay_source));
//    APP_ITM.borrow(cs).replace(Some(cp.ITM));
  });

}

#[cfg(feature = "stm32h7x")]
fn setup_peripherals_h7x()  {
  let dp = pac::Peripherals::take().unwrap();
  let cp = cortex_m::Peripherals::take().unwrap();

  // Set up the system clock
  let rcc = dp.RCC.constrain();

  let pwr = dp.PWR.constrain();
  let vos = pwr.freeze();

  //use the existing sysclk
  let mut ccdr = rcc.freeze(vos, &dp.SYSCFG);
  let clocks = ccdr.clocks;
  let delay_source =  p_hal::delay::Delay::new(cp.SYST, clocks);

  let gpiob = dp.GPIOB.split(&mut ccdr.ahb4);

  let mut user_led1 = gpiob.pb0.into_push_pull_output();
  //set initial states of user LEDs
  user_led1.set_high().unwrap();

  // setup i2c1 and imu driver
  let scl = gpiob.pb8.into_alternate_af4()
      //.internal_pull_up(true)
      .set_open_drain();
  let sda = gpiob.pb9.into_alternate_af4()
      //.internal_pull_up(true)
      .set_open_drain();
  //  let imu_i2c_port = dp.I2C1.i2c((scl, sda), 400.khz(), clocks);
  let imu_i2c_port = p_hal::i2c::I2c::i2c1(dp.I2C1, (scl, sda), 100.khz(), &ccdr);//TODO raise to 400
  let imu_driver = BNO080::new_with_interface(imu_i2c_port);

  //store shared peripherals
  interrupt::free(|cs| {
    USER_LED_1.borrow(cs).replace(Some(user_led1));
    IMU_DRIVER.borrow(cs).replace(Some(imu_driver));
    APP_DELAY_SOURCE.borrow(cs).replace(Some(delay_source));
  });

}

/// Setup peripherals such as GPIO
fn setup_peripherals()  {

  #[cfg(feature = "stm32f3x")]
  setup_peripherals_f3x();

  #[cfg(feature = "stm32f4x")]
  setup_peripherals_f4x();

  #[cfg(feature = "stm32h7x")]
  setup_peripherals_h7x();

}

/// Configure and start the RTOS
fn configure_rtos() {

  let _rc = cmsis_rtos2::rtos_kernel_initialize();
  d_println!(get_debug_log(), "kernel_initialize rc: {}", _rc);

  let _tick_hz = cmsis_rtos2::rtos_kernel_get_tick_freq_hz();
  d_println!(get_debug_log(), "tick_hz : {}", _tick_hz);

  let _sys_timer_hz = cmsis_rtos2::rtos_kernel_get_sys_timer_freq_hz();
  d_println!(get_debug_log(), "sys_timer_hz : {}", _sys_timer_hz);

}

fn start_rtos() {
  let _rc = cmsis_rtos2::rtos_kernel_start();
  d_println!(get_debug_log(), "kernel_start rc: {}", _rc);

  //d_println!(get_debug_log(),"RTOS done!");
}

#[entry]
fn main() -> ! {

  setup_peripherals();
  configure_rtos();

  // start the blinking task
  setup_blink_task();

  // initialize the IMU task
  setup_imu();

  start_rtos();

  loop {
    //typically this is never called after the rtos starts
    d_println!(get_debug_log(),"mainloop");
  }

}
