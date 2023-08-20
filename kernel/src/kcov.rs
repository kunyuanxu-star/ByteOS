use alloc::sync::Arc;
use sync::Mutex;
use vfscore::INodeInterface;
use core::{sync::atomic::AtomicBool, arch::asm};

pub struct Fuz{
    inner: Mutex<(usize, usize)>
}

impl Fuz {
    pub fn new() -> Arc<Fuz> {
        Arc::new(Self {
            inner: Mutex::new((0, 0))
        })
    }
}

impl INodeInterface for Fuz {
    fn readat(&self, _offset: usize, _buffer: &mut [u8]) -> vfscore::VfsResult<usize> {
        Ok(0)    
    }

    fn writeat(&self, _offset: usize, _buffer: &[u8]) -> vfscore::VfsResult<usize> {
        Ok(0)
    }

    fn ioctl(&self, command: usize, _arg: usize) -> vfscore::VfsResult<usize> {
        log::info!("ioctl: {} arg: {}", command, _arg);
        const ENABLE_FUZ: usize = 25444;
        const DISABLE_FUZ: usize = 25445;
        const INIT_FUZ_TRACE: usize = 2148033281;
        match command {
            ENABLE_FUZ => {
                let (addr, len) = *self.inner.lock();
                enable_fuz(addr, len);
                Ok(0)
            }
            DISABLE_FUZ => {
                disable_fuz();
                Ok(0)
            }
            INIT_FUZ_TRACE => {
                let (addr, len) = *self.inner.lock();
                init_fuz(addr, len);
                Ok(0)
            }
            _ => Err(vfscore::VfsError::NotSupported)
        }
    }

    fn after_mmap(&self, _addr: usize, _size: usize) -> vfscore::VfsResult<()> {
        log::info!("after_mmap: {}, size: {}", _addr, _size);
        *self.inner.lock() = (_addr, _size);
        Ok(())
    }
}

pub static IS_FUZZING: AtomicBool = AtomicBool::new(false);

pub struct FuzzItem {
    pub ip: usize,
    pub tp: usize,
    pub arg1: usize,
    pub arg2: usize,
}

pub static FUZ_RECORDS: Mutex<(usize, usize)> = Mutex::new((0, 0));

pub fn enable_fuz(addr: usize, len: usize) {
    IS_FUZZING.store(true, core::sync::atomic::Ordering::Relaxed);
    log::info!("enable_fuzing: {:#x}, {:#x}", addr, len);
    *FUZ_RECORDS.lock() = (addr, len);
}

pub fn disable_fuz() {
    IS_FUZZING.store(false, core::sync::atomic::Ordering::Relaxed);
    *FUZ_RECORDS.lock() = (0, 0);
}

pub fn init_fuz(addr: usize, len: usize) {
    *FUZ_RECORDS.lock() = (addr, len);
}

pub fn write_fuz(data: &str) {
    if !IS_FUZZING.fetch_or(false, core::sync::atomic::Ordering::Relaxed) {
        return;
    }
    let ra: usize;
    unsafe {
        asm!("", out("ra") ra)
    }
    // log::error!("ra: {:#x}", ra);
    let (addr, len) = *FUZ_RECORDS.lock();
    if addr == 0 || len == 0 {
        return;
    }
    unsafe {
        let n = addr as *mut usize;
        let wptr = (addr as *mut usize).add(*n + 1);
        if wptr as usize >= addr + len {
            return;
        }
        wptr.write_volatile(ra);
        *n +=1;
    }
    println!("{}", data);
}

pub macro fuz() {
    fn f() {}
    fn type_name_of<T>(_: T) -> &'static str {
        core::any::type_name::<T>()
    }
    let name = type_name_of(f);
    $crate::kcov::write_fuz(&format!("{}\n{}: {}", &name[..name.len() - 16], file!(), line!()))
}
