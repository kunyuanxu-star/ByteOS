#![no_main]
#![no_std]
#![feature(exclusive_range_pattern)]
#![feature(drain_filter)]

#[macro_use]
extern crate logging;
#[macro_use]
extern crate alloc;

mod modules;
mod syscall;
mod tasks;

use arch::{shutdown, Context, TrapType, reboot};
use devices;
use executor::get_current_task;
use frame_allocator;
use hal;
use kalloc;
use log::warn;
use panic_handler as _;

use crate::tasks::kernel::kernel_interrupt;

pub fn kernel_int(_cx: &mut Context, trap_type: TrapType) {
    match trap_type {
        TrapType::StorePageFault(addr) | TrapType::InstructionPageFault(addr) => {
            // judge whether it is trigger by a user_task handler.
        }
        _ => {
            // warn!("trap_type: {:?}  context: {:#x?}", trap_type, cx);
            warn!("kernel_interrupt");
        }
    };
}

#[no_mangle]
fn main(hart_id: usize, device_tree: usize) {
    // if hart_id != 0 {
    //     loop {}
    // }

    extern "C" {
        fn start();
        fn end();
    }

    let str = include_str!("banner.txt");
    println!("{}", str);

    // initialize logging module
    logging::init();

    info!(
        "program size: {}KB",
        (end as usize - start as usize) / 0x400
    );

    // initialize interrupt
    hal::interrupt::init();
    hal::interrupt::reg_kernel_int(kernel_int);

    // print boot info
    info!("booting at kernel {}", hart_id);

    // initialize kernel alloc module
    kalloc::init();

    // initialize device settings
    devices::init_device(device_tree);

    // initialize frame allocator
    frame_allocator::init();

    // get devices and init
    devices::prepare_devices();

    // init cv1811 sdcard
    // this is a test function for developing
    // cv1811_sd::init();

    // // initialize filesystem
    // fs::init();

    // // init kernel threads and async executor
    // tasks::init();

    println!("Task All Finished!");
    shutdown();
}
