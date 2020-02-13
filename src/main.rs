#![deny(warnings)]
//#![deny(unsafe_code)]
#![no_main]
#![no_std]

//extern crate panic_itm;

extern crate panic_semihosting;


use core::cell::RefCell;
use cortex_m::interrupt::{self, Mutex};

#[cfg(feature = "stm32h7x")]
use stm32h7xx_hal as processor_hal;

#[cfg(feature = "stm32f4x")]
use stm32f4xx_hal as processor_hal;

use processor_hal::prelude::*;
use processor_hal::stm32 as pac;
use pac::I2C1;
//use pac::DWT;


#[macro_use]
extern crate cortex_m_rt;

use cortex_m_rt::{entry, ExceptionFrame};

use processor_hal::hal::digital::v2::ToggleableOutputPin;


use cmsis_rtos2;

#[cfg(feature = "stm32f4x")]
#[allow(non_upper_case_globals)]
#[no_mangle]
//pub static SystemCoreClock: u32 =  16_000_000; //from stm32f4xx_hal::rcc::HSI
 pub static SystemCoreClock: u32 =  25_000_000; //from stm32f4xx_hal::rcc::HSI
//Can use 25_000_000 on an stm32f401 board with 25 MHz xtal

#[cfg(feature = "stm32h7x")]
#[allow(non_upper_case_globals)]
#[no_mangle]
pub static SystemCoreClock: u32 = 48_000_000; //stm32h743 HSI (48 MHz)

#[cfg(debug_assertions)]
use cortex_m_log::{println};

use cortex_m_log::{d_println};

// FOR itm mode:
//use cortex_m_log::{
//  destination::Itm, printer::itm::InterruptSync as InterruptSyncItm,
//};

//#[cfg(debug_assertions)]
//use cortex_m_log::printer::semihosting;

//#[cfg(debug_assertions)]
//use cortex_m_semihosting;


use processor_hal::rcc::Clocks;

use processor_hal::gpio::GpioExt;
use processor_hal::rcc::RccExt;

use core::ops::{DerefMut};

use core::ptr::{null, null_mut};
//use cmsis_rtos2::{ osMessageQueueId_t};
use bno080::*;

const IMU_REPORTING_RATE_HZ: u16 = 500;
const IMU_REPORTING_INTERVAL_MS: u16 = (1000 / IMU_REPORTING_RATE_HZ) ;

type ImuDriverType = bno080::BNO080<processor_hal::i2c::I2c<I2C1,
  (processor_hal::gpio::gpiob::PB8<processor_hal::gpio::Alternate<processor_hal::gpio::AF4>>,
   processor_hal::gpio::gpiob::PB9<processor_hal::gpio::Alternate<processor_hal::gpio::AF4>>)
>>;




#[cfg(debug_assertions)]
type DebugLog = cortex_m_log::printer::dummy::Dummy;
//type DebugLog = cortex_m_log::printer::semihosting::Semihosting<cortex_m_log::modes::InterruptFree, cortex_m_semihosting::hio::HStdout>;

#[cfg(feature = "stm32f4x")]
type GpioTypeUserLed1 =  processor_hal::gpio::gpioc::PC13<processor_hal::gpio::Output<processor_hal::gpio::PushPull>>;

#[cfg(feature = "stm32h7x")]
type GpioTypeUserLed1 =  processor_hal::gpio::gpiob::PB0<processor_hal::gpio::Output<processor_hal::gpio::PushPull>>;


static APP_CLOCKS:  Mutex<RefCell< Option< Clocks >>> = Mutex::new(RefCell::new(None));
static USER_LED_1:  Mutex<RefCell<Option< GpioTypeUserLed1>>> = Mutex::new(RefCell::new(None));

static APP_DELAY_SOURCE: Mutex<RefCell<Option<  processor_hal::delay::Delay >>> = Mutex::new(RefCell::new(None));
static IMU_DRIVER: Mutex<RefCell<Option< ImuDriverType>>> = Mutex::new(RefCell::new(None));


//static mut GLOBAL_QUEUE_HANDLE: Option< osMessageQueueId_t  > = None;




// cortex-m-rt is setup to call DefaultHandler for a number of fault conditions
// we can override this in debug mode for handy debugging
#[exception]
fn DefaultHandler(_irqn: i16) {
  d_println!(get_debug_log(), "IRQn = {}", _irqn);
}


// cortex-m-rt calls this for serious faults.  can set a breakpoint to debug
#[exception]
fn HardFault(_ef: &ExceptionFrame) -> ! {
  loop {

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


//#[no_mangle]
//extern "C" fn task_blink(_arg: *mut cty::c_void) {
//  let mq_id:osMessageQueueId_t = unsafe { GLOBAL_QUEUE_HANDLE.unwrap() } ;
//  let mut send_buf: [u8; 10] = [0; 10];
//  loop {
//    cmsis_rtos2::rtos_os_msg_queue_put(
//      mq_id as osMessageQueueId_t,
//      send_buf.as_ptr() as *const cty::c_void,
//      1,
//      250);
//
//    send_buf[0] = (send_buf[0] + 1) % 255;
//  }
//
//}

//#[no_mangle]
//extern "C" fn task1_cb(_arg: *mut cty::c_void) {
//  let mq_id:osMessageQueueId_t = unsafe { GLOBAL_QUEUE_HANDLE.unwrap() } ;
//  let mut send_buf: [u8; 10] = [0; 10];
//  loop {
//    cmsis_rtos2::rtos_os_msg_queue_put(
//      mq_id as osMessageQueueId_t,
//      send_buf.as_ptr() as *const cty::c_void,
//      1,
//      250);
//
//    send_buf[0] = (send_buf[0] + 1) % 255;
//  }
//
//}

//#[no_mangle]
//extern "C" fn task2_cb(_arg: *mut cty::c_void) {
//  let mq_id:osMessageQueueId_t = unsafe { GLOBAL_QUEUE_HANDLE.unwrap() } ;
//  let mut recv_buf: [u8; 10] = [0; 10];
//
//  loop {
//    let rc = cmsis_rtos2::rtos_os_msg_queue_get(mq_id,
//                                                recv_buf.as_mut_ptr() as *mut cty::c_void,
//                                                null_mut(), 100);
//    if 0 == rc {
//      toggle_leds();
//      cmsis_rtos2::rtos_os_delay(50);
//    }
//
//  }
//
//}


fn setup_imu() {
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
    setup_repeated_imu_timer();
  }
}


/// Second-stage initialization callback
#[no_mangle]
extern "C" fn task_kicker(_arg: *mut cty::c_void) {
  // start the blinking task
  setup_repeated_blink_timer();
  // initialize the IMU
  setup_imu();
}

#[no_mangle]
extern "C" fn repeated_blink_cb(_arg: *mut cty::c_void) {
  toggle_leds();
}


pub fn setup_repeated_blink_timer() {
  let tid = cmsis_rtos2::rtos_os_timer_new(
    Some(repeated_blink_cb),
    cmsis_rtos2::osTimerType_t_osTimerPeriodic,
    null_mut(),
    null(),
  );

  if tid.is_null() {
    d_println!(get_debug_log(), "setup_repeating_timer failed...");
  }
  else {
    let rc = cmsis_rtos2::rtos_os_timer_start(tid, 50);
    if 0 != rc {
      d_println!(get_debug_log(), "rtos_os_timer_start failed {:?}", rc);
    }
    else {
      d_println!(get_debug_log(),"blink timer: {:?}", tid);
    }
  }

}


/// Give some time to the IMU driver to process sensor reports
fn spin_imu_driver() {
  interrupt::free(|cs| {
    if let Some(ref mut imu_driver) = IMU_DRIVER.borrow(cs).borrow_mut().deref_mut() {
      imu_driver.handle_all_messages();
    }
  });
}


#[no_mangle]
extern "C" fn repeated_imu_cb(_arg: *mut cty::c_void) {
  spin_imu_driver();
}


/// Start a timer that will repeatedly spin the imu driver
pub fn setup_repeated_imu_timer() {
  let tid = cmsis_rtos2::rtos_os_timer_new(
    Some(repeated_imu_cb),
    cmsis_rtos2::osTimerType_t_osTimerPeriodic,
    null_mut(),
    null(),
  );

  if tid.is_null() {
    d_println!(get_debug_log(), "setup_repeated_imu_timer failed...");
  }
  else {
    let rc = cmsis_rtos2::rtos_os_timer_start(tid, 50);
    if 0 != rc {
      d_println!(get_debug_log(), "rtos_os_timer_start failed {:?}", rc);
    }
    else {
      d_println!(get_debug_log(),"imu timer: {:?}", tid);
    }
  }

}


/// Start a task that will perform second-stage initialization
pub fn setup_kicker_thread() {
  let tid = cmsis_rtos2::rtos_os_thread_new(
    Some(task_kicker),
    null_mut(),
    null(),
  );
  if tid.is_null() {
    d_println!(get_debug_log(), "rtos_os_thread_new failed!");
    return;
  }
}

//pub fn setup_ping_pong_tasks() {
//
//// create a shared msg queue
//  let mq = cmsis_rtos2::rtos_os_msg_queue_new(10, 4, null());
//  if mq.is_null() {
//   d_println!(get_debug_log(), "rtos_os_msg_queue_new failed");
//   return;
//  }
//
//  unsafe {
//   GLOBAL_QUEUE_HANDLE = Some(mq);
//  }
//
//  // We don't pass context to the default task here, since that involves problematic
//  // casting to/from C void pointers; instead, we use global static context.
//  let thread1_id = cmsis_rtos2::rtos_os_thread_new(
//    Some(task_blink),
//    null_mut(),
//    null(),
//  );
//  if thread1_id.is_null() {
//    d_println!(get_debug_log(), "rtos_os_thread_new failed!");
//    return;
//  }
//
//  let thread2_id = cmsis_rtos2::rtos_os_thread_new(
//    Some(task2_cb),
//    null_mut(),
//    null(),
//  );
//  if thread2_id.is_null() {
//    d_println!(get_debug_log(), "rtos_os_thread_new failed!");
//    return;
//  }
//}

// Setup peripherals such as GPIO
fn setup_peripherals()  {
  //d_print!(get_debug_log(), "setup_peripherals...");

  let dp = pac::Peripherals::take().unwrap();
  let cp = cortex_m::Peripherals::take().unwrap();
//  DWT.enable_cycle_counter();

  // Set up the system clock
  let rcc = dp.RCC.constrain();
  #[cfg(feature = "stm32f4x")]
  let clocks = rcc.cfgr.use_hse(SystemCoreClock.hz()).freeze();
  #[cfg(not(feature = "stm32f4x"))]

  let clocks = rcc.cfgr.freeze();

//  let clocks = rcc.cfgr.sysclk(16.mhz()).freeze();
  let delay_source =  processor_hal::delay::Delay::new(cp.SYST, clocks);

  let gpiob = dp.GPIOB.split();
  let gpioc = dp.GPIOC.split();
  let mut user_led1 = gpioc.pc13.into_push_pull_output();

  //set initial states of user LEDs
  user_led1.set_high().unwrap();

  // setup i2c1 and imu driver
  let scl = gpiob.pb8.into_alternate_af4().internal_pull_up(true).set_open_drain();
  let sda = gpiob.pb9.into_alternate_af4().internal_pull_up(true).set_open_drain();
  let imu_i2c_port = processor_hal::i2c::I2c::i2c1(dp.I2C1, (scl, sda), 400.khz(), clocks);
  let imu_driver = BNO080::new(imu_i2c_port);

  //store shared peripherals
  interrupt::free(|cs| {
    APP_CLOCKS.borrow(cs).replace(Some(clocks));
    USER_LED_1.borrow(cs).replace(Some(user_led1));
    IMU_DRIVER.borrow(cs).replace(Some(imu_driver));
    APP_DELAY_SOURCE.borrow(cs).replace(Some(delay_source));
  });

}


fn setup_rtos() {

  let _rc = cmsis_rtos2::rtos_kernel_initialize();
//  d_println!(get_debug_log(), "kernel_initialize rc: {}", _rc);

  let _tick_hz = cmsis_rtos2::rtos_kernel_get_tick_freq_hz();
//  d_println!(get_debug_log(), "tick_hz : {}", _tick_hz);

  let _sys_timer_hz = cmsis_rtos2::rtos_kernel_get_sys_timer_freq_hz();
//  d_println!(get_debug_log(), "sys_timer_hz : {}", _sys_timer_hz);

  setup_kicker_thread();

//  setup_default_threads();

  let _rc = cmsis_rtos2::rtos_kernel_start();
  d_println!(get_debug_log(), "kernel_start rc: {}", _rc);

  //d_println!(get_debug_log(),"RTOS done!");

}

#[entry]
fn main() -> ! {

  setup_peripherals();
  setup_rtos();

  loop {
    //typically this is never called
    d_println!(get_debug_log(),"mainloop");
    //one hz heartbeat
    cmsis_rtos2::rtos_os_delay(1000);
  }

}
