#![deny(warnings)]
//#![deny(unsafe_code)]
#![no_main]
#![no_std]

//extern crate panic_itm;

extern crate panic_semihosting;


use core::cell::RefCell;
use cortex_m::interrupt::{self, Mutex};

//use stm32h7xx_hal as processor_hal;
use stm32f4xx_hal as processor_hal;

//use stm32h7xx_hal::pac as pac;


#[macro_use]
extern crate cortex_m_rt;

use cortex_m_rt::{entry, ExceptionFrame};

//use processor_hal::hal::digital::v2::OutputPin;
use processor_hal::hal::digital::v2::ToggleableOutputPin;
//use processor_hal::hal::digital::v2::InputPin;


use cmsis_rtos2;

#[allow(non_upper_case_globals)]
#[no_mangle]
pub static SystemCoreClock: u32 = 16_000_000; //or use stm32f4xx_hal rcc::HSI
//Can use 25_000_000 on an stm32f401 board with 25 MHz xtal
// 48_000_000 for stm32h743 HSI (48 MHz)

#[cfg(debug_assertions)]
use cortex_m_log::{print, println};

use cortex_m_log::{d_print, d_println};
// FOR itm mode:
//use cortex_m_log::{
//  destination::Itm, printer::itm::InterruptSync as InterruptSyncItm,
//};
#[cfg(debug_assertions)]
use cortex_m_log::printer::semihosting;

#[cfg(debug_assertions)]
use cortex_m_semihosting;


use processor_hal::rcc::Clocks;

use processor_hal::gpio::GpioExt;
use processor_hal::rcc::RccExt;

use core::ops::{DerefMut};

use processor_hal::{prelude::*, stm32};
use core::ptr::{null, null_mut};
use cmsis_rtos2::{ osMessageQueueId_t};

#[cfg(debug_assertions)]
type DebugLog = cortex_m_log::printer::semihosting::Semihosting<cortex_m_log::modes::InterruptFree, cortex_m_semihosting::hio::HStdout>;

//TODO this kind of hardcoding is not ergonomic

type GpioTypeUserLed1 =  processor_hal::gpio::gpioc::PC13<processor_hal::gpio::Output<processor_hal::gpio::PushPull>>;

static APP_CLOCKS:  Mutex<RefCell< Option< Clocks >>> = Mutex::new(RefCell::new(None));
static USER_LED_1:  Mutex<RefCell<Option< GpioTypeUserLed1>>> = Mutex::new(RefCell::new(None));

static mut GLOBAL_QUEUE_HANDLE: Option< osMessageQueueId_t  > = None;

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
  semihosting::InterruptFree::<_>::stdout().unwrap()
}



// Toggle the user leds from their prior state
fn toggle_leds() {
  interrupt::free(|cs| {
    if let Some(ref mut led1) = USER_LED_1.borrow(cs).borrow_mut().deref_mut() {
      led1.toggle().unwrap();
    }
  });
}


#[no_mangle]
extern "C" fn task_blink(_arg: *mut cty::c_void) {
  let mq_id:osMessageQueueId_t = unsafe { GLOBAL_QUEUE_HANDLE.unwrap() } ;
  let mut send_buf: [u8; 10] = [0; 10];
  loop {
    cmsis_rtos2::rtos_os_msg_queue_put(
      mq_id as osMessageQueueId_t,
      send_buf.as_ptr() as *const cty::c_void,
      1,
      250);

    send_buf[0] = (send_buf[0] + 1) % 255;
  }

}

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

#[no_mangle]
extern "C" fn task2_cb(_arg: *mut cty::c_void) {
  let mq_id:osMessageQueueId_t = unsafe { GLOBAL_QUEUE_HANDLE.unwrap() } ;
  let mut recv_buf: [u8; 10] = [0; 10];

  loop {
    let rc = cmsis_rtos2::rtos_os_msg_queue_get(mq_id,
                                                recv_buf.as_mut_ptr() as *mut cty::c_void,
                                                null_mut(), 100);
    if 0 == rc {
      toggle_leds();
      cmsis_rtos2::rtos_os_delay(50);
    }

  }

}

#[no_mangle]
extern "C" fn repeated_blink_cb(_arg: *mut cty::c_void) {
  toggle_leds();
  //d_print!(get_debug_log(), ".");
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


#[no_mangle]
extern "C" fn repeated_imu_cb(_arg: *mut cty::c_void) {
  //toggle_leds();
  //d_print!(get_debug_log(), ".");
}


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

pub fn setup_default_threads() {

// create a shared msg queue
  let mq = cmsis_rtos2::rtos_os_msg_queue_new(10, 4, null());
  if mq.is_null() {
   d_println!(get_debug_log(), "rtos_os_msg_queue_new failed");
   return;
  }

  unsafe {
   GLOBAL_QUEUE_HANDLE = Some(mq);
  }

  // We don't pass context to the default task here, since that involves problematic
  // casting to/from C void pointers; instead, we use global static context.
  let thread1_id = cmsis_rtos2::rtos_os_thread_new(
    Some(task_blink),
    null_mut(),
    null(),
  );
  if thread1_id.is_null() {
    d_println!(get_debug_log(), "rtos_os_thread_new failed!");
    return;
  }

  let thread2_id = cmsis_rtos2::rtos_os_thread_new(
    Some(task2_cb),
    null_mut(),
    null(),
  );
  if thread2_id.is_null() {
    d_println!(get_debug_log(), "rtos_os_thread_new failed!");
    return;
  }
}

// Setup peripherals such as GPIO
fn setup_peripherals()  {
  //d_print!(get_debug_log(), "setup_peripherals...");

  let dp = stm32::Peripherals::take().unwrap();

  let gpioc = dp.GPIOC.split();
  let mut user_led1 = gpioc.pc13.into_push_pull_output();

  // Set up the system clock at 16 MHz
  let rcc = dp.RCC.constrain();
  let clocks = rcc.cfgr.freeze();
//  let clocks = rcc.cfgr.sysclk(16.mhz()).freeze();

  //set initial states of user LEDs
  user_led1.set_high().unwrap();

  //store shared peripherals
  interrupt::free(|cs| {
    APP_CLOCKS.borrow(cs).replace(Some(clocks));
    USER_LED_1.borrow(cs).replace(Some(user_led1));
  });

  //d_println!(get_debug_log(), "done!");

}


fn setup_rtos() {
//  d_println!(get_debug_log(), "Setup RTOS...");

  let _rc = cmsis_rtos2::rtos_kernel_initialize();
//  d_println!(get_debug_log(), "kernel_initialize rc: {}", _rc);

  let _tick_hz = cmsis_rtos2::rtos_kernel_get_tick_freq_hz();
//  d_println!(get_debug_log(), "tick_hz : {}", _tick_hz);

  let _sys_timer_hz = cmsis_rtos2::rtos_kernel_get_sys_timer_freq_hz();
//  d_println!(get_debug_log(), "sys_timer_hz : {}", _sys_timer_hz);


  setup_repeated_imu_timer();
  setup_repeated_blink_timer();

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
    //cmsis_rtos2::rtos_os_thread_yield();
    //one hz heartbeat
    cmsis_rtos2::rtos_os_delay(1000);
    d_print!(get_debug_log(),".");
  }

}
