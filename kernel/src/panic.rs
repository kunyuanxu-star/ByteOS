use arch::shutdown;
use backtrace::backtrace;
use core::panic::PanicInfo;

// 程序遇到错误
#[panic_handler]
fn panic_handler(info: &PanicInfo) -> ! {
    println!("\x1b[1;31mpanic: '{}'\x1b[0m", info.message().unwrap());
    backtrace();
    println!("!TEST FINISH!");
    // loop {}
    shutdown()
}
