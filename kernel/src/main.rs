#![no_main]
#![no_std]
#![feature(exclusive_range_pattern)]
#![feature(drain_filter)]

#[macro_use]
extern crate logging;
#[macro_use]
extern crate alloc;

mod syscall;
mod tasks;

use devices;
use frame_allocator;
use hal;
use kalloc;
use panic_handler as _;


fn clear_bss() {
    extern "C" {
        fn sbss();
        fn ebss();
    }
    unsafe {
        core::slice::from_raw_parts_mut(sbss as usize as *mut u8, ebss as usize - sbss as usize)
            .fill(0);
    }
}


#[no_mangle]
fn main(hart_id: usize, device_tree: usize) {
    if hart_id != 0 {
        loop {}
    }

    clear_bss();

    let str = include_str!("banner.txt");
    println!("{}", str);

    // initialize logging module
    logging::init();

    // initialize interrupt
    hal::interrupt::init();

    // print boot info
    println!("booting at kernel {}", hart_id);

    // initialize kernel alloc module
    kalloc::init();

    // initialize device settings
    devices::init_device(device_tree);

    // initialize frame allocator
    frame_allocator::init();

    // get devices and init
    devices::prepare_devices();

    // initialize filesystem
    // fs::init();

    // init kernel threads and async executor
    tasks::init();

    println!("Task All Finished!");
}
