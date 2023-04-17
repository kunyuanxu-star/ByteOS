#![no_std]
#![no_main]
#![feature(naked_functions)]
#![feature(asm_const)]
#![feature(once_cell)]

#[macro_use]
extern crate log;

#[cfg(target_arch = "riscv64")]
mod riscv64;

#[cfg(target_arch = "riscv64")]
pub use riscv64::*;

pub struct IntTable {
    pub timer: fn(),
}

pub trait ContextOps {
    fn set_sp(&mut self, sp: usize);
    fn sp(&self) -> usize;
    fn set_ra(&mut self, ra: usize);
    fn ra(&self) -> usize;
    fn set_sepc(&mut self, sepc: usize);
    fn sepc(&self) -> usize;

    fn syscall_number(&self) -> usize;
    fn args(&self) -> &[usize];
    fn syscall_ok(&mut self);

    fn set_ret(&mut self, ret: usize);
}

extern "Rust" {
    fn interrupt_table() -> IntTable;
}

pub enum TrapType {
    Breakpoint,
    UserEnvCall,
    Time,
    Unknown,
    StorePageFault(usize),
}