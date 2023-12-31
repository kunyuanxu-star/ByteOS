use riscv::register::sstatus;

use crate::ContextOps;

#[repr(C)]
#[derive(Debug, Clone)]
// 上下文
pub struct Context {
    pub x: [usize; 32], // 32 个通用寄存器
    pub sstatus: usize,
    pub sepc: usize,
    pub fsx: [usize; 2],
}

impl Context {
    // 创建上下文信息
    #[inline]
    pub fn new() -> Self {
        Context {
            x: [0usize; 32],
            sstatus: sstatus::read().bits(),
            sepc: 0,
            fsx: [0; 2],
        }
    }
    // 从另一个上下文复制
    #[inline]
    pub fn clone_from(&mut self, target: &Self) {
        for i in 0..32 {
            self.x[i] = target.x[i];
        }

        self.sstatus = target.sstatus;
        self.sepc = target.sepc;
        self.fsx = target.fsx;
    }
}

impl ContextOps for Context {
    #[inline]
    fn set_sp(&mut self, sp: usize) {
        self.x[2] = sp;
    }

    #[inline]
    fn sp(&self) -> usize {
        self.x[2]
    }
    #[inline]
    fn set_ra(&mut self, ra: usize) {
        self.x[1] = ra;
    }

    #[inline]
    fn ra(&self) -> usize {
        self.x[1]
    }

    #[inline]
    fn set_sepc(&mut self, sepc: usize) {
        self.sepc = sepc;
    }

    #[inline]
    fn sepc(&self) -> usize {
        self.sepc
    }

    #[inline]
    fn syscall_number(&self) -> usize {
        self.x[17]
    }

    #[inline]
    fn args(&self) -> [usize; 7] {
        self.x[10..17].try_into().expect("args slice force convert")
    }

    #[inline]
    fn syscall_ok(&mut self) {
        self.sepc += 4;
    }

    fn set_ret(&mut self, ret: usize) {
        self.x[10] = ret;
    }

    fn set_arg0(&mut self, ret: usize) {
        self.x[10] = ret;
    }

    fn set_arg1(&mut self, ret: usize) {
        self.x[11] = ret;
    }

    fn set_arg2(&mut self, ret: usize) {
        self.x[12] = ret;
    }

    fn set_tp(&mut self, tp: usize) {
        self.x[4] = tp;
    }

    fn tp(&self) -> usize {
        self.x[4]
    }

    fn clear(&mut self) {
        self.x.fill(0);
        self.sepc = 0;
        self.sstatus = sstatus::read().bits();
    }

    #[inline]
    fn set_tls(&mut self, tls: usize) {
        self.x[4] = tls;
    }
}
