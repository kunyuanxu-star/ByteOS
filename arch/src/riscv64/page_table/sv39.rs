use core::arch::{asm, riscv64::sfence_vma};

use bitflags::bitflags;

use crate::{
    current_page_table, sigtrx::get_trx_mapping, PhysAddr, PhysPage, VirtAddr, VirtPage,
    PAGE_FRAME_BASE, PAGE_ITEM_COUNT, PAGE_SIZE,
};

#[derive(Copy, Clone, Debug)]
pub struct PTE(usize);

impl PTE {
    #[inline]
    pub const fn new() -> Self {
        Self(0)
    }

    #[inline]
    pub const fn from_ppn(ppn: usize, flags: PTEFlags) -> Self {
        // let flags = flags.union(PTEFlags::D);
        let mut flags = flags;
        if flags.contains(PTEFlags::R) | flags.contains(PTEFlags::X) {
            flags = flags.union(PTEFlags::A)
        }
        if flags.contains(PTEFlags::W) {
            flags = flags.union(PTEFlags::D)
        }
        // TIPS: This is prepare for the extend bits of T-HEAD C906
        #[cfg(feature = "board-cv1811h")]
        if flags.contains(PTEFlags::G) && ppn == 0x8_0000 {
            Self(
                ppn << 10
                    | flags
                        .union(PTEFlags::C)
                        .union(PTEFlags::B)
                        .union(PTEFlags::K)
                        .bits() as usize,
            )
        } else if flags.contains(PTEFlags::G) && ppn == 0 {
            Self(ppn << 10 | flags.union(PTEFlags::SE).union(PTEFlags::SO).bits() as usize)
        } else {
            Self(ppn << 10 | flags.union(PTEFlags::C).bits() as usize)
        }

        #[cfg(not(feature = "board-cv1811h"))]
        Self(ppn << 10 | flags.bits() as usize)
    }

    #[inline]
    pub const fn from_addr(addr: usize, flags: PTEFlags) -> Self {
        Self::from_ppn(addr >> 12, flags)
    }

    #[inline]
    pub const fn to_ppn(&self) -> PhysPage {
        PhysPage((self.0 >> 10) & ((1 << 29) - 1))
    }

    #[inline]
    pub fn set(&mut self, ppn: usize, flags: PTEFlags) {
        self.0 = (ppn << 10) | flags.bits() as usize;
    }

    #[inline]
    pub const fn flags(&self) -> PTEFlags {
        PTEFlags::from_bits_truncate((self.0 & 0xff) as u64)
    }

    #[inline]
    pub const fn is_valid(&self) -> bool {
        self.flags().contains(PTEFlags::V) && self.0 > u8::MAX as usize
    }

    /// 判断是否是大页
    ///
    /// 大页判断条件 V 位为 1, R/W/X 位至少有一个不为 0
    /// PTE 页表范围 1G(0x4000_0000) 2M(0x20_0000) 4K(0x1000)
    #[inline]
    pub fn is_huge(&self) -> bool {
        return self.flags().contains(PTEFlags::V)
            && (self.flags().contains(PTEFlags::R)
                || self.flags().contains(PTEFlags::W)
                || self.flags().contains(PTEFlags::X));
    }
}

bitflags! {
    #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
    pub struct PTEFlags: u64 {
        const NONE = 0;
        const V = 1 << 0;
        const R = 1 << 1;
        const W = 1 << 2;
        const X = 1 << 3;
        const U = 1 << 4;
        const G = 1 << 5;
        const A = 1 << 6;
        const D = 1 << 7;

        #[cfg(feature = "board-cv1811h")]
        const SO = 1 << 63;
        #[cfg(feature = "board-cv1811h")]
        const C = 1 << 62;
        #[cfg(feature = "board-cv1811h")]
        const B = 1 << 61;
        #[cfg(feature = "board-cv1811h")]
        const K = 1 << 60;
        #[cfg(feature = "board-cv1811h")]
        const SE = 1 << 59;

        const AD = Self::A.bits() | Self::D.bits();
        const VRW   = Self::V.bits() | Self::R.bits() | Self::W.bits();
        const VRWX  = Self::V.bits() | Self::R.bits() | Self::W.bits() | Self::X.bits();
        const UVRX = Self::U.bits() | Self::V.bits() | Self::R.bits() | Self::X.bits();
        const ADUVRX = Self::A.bits() | Self::D.bits() | Self::U.bits() | Self::V.bits() | Self::R.bits() | Self::X.bits();
        const UVRWX = Self::U.bits() | Self::VRWX.bits();
        const UVRW = Self::U.bits() | Self::VRW.bits();
        const GVRWX = Self::G.bits() | Self::VRWX.bits();
        const ADVRWX = Self::A.bits() | Self::D.bits() | Self::G.bits() | Self::VRWX.bits();
        const ADGVRWX = Self::A.bits() | Self::D.bits() | Self::G.bits() | Self::VRWX.bits();
    }
}

#[derive(Debug, Clone, Copy)]
pub struct PageTable(pub(crate) PhysAddr);

impl PageTable {
    pub fn new(addr: PhysAddr) -> Self {
        let page_table = Self(addr);
        page_table.restore();
        page_table
    }

    #[inline]
    pub fn from_ppn(ppn: PhysPage) -> Self {
        Self::new(PhysAddr(ppn.0 << 12))
    }

    #[inline]
    pub fn restore(&self) {
        let arr = self.get_pte_list();
        arr[0x100] = PTE::from_addr(0x0000_0000, PTEFlags::ADGVRWX);
        arr[0x101] = PTE::from_addr(0x4000_0000, PTEFlags::ADGVRWX);
        arr[0x102] = PTE::from_addr(0x8000_0000, PTEFlags::ADGVRWX);
        arr[0x104] = PTE::from_addr(get_trx_mapping(), PTEFlags::V);
        arr[0x106] = PTE::from_addr(0x8000_0000, PTEFlags::ADGVRWX);
        arr[0..0x100].fill(PTE::from_addr(0, PTEFlags::NONE));
    }

    #[inline]
    pub fn get_pte_list(&self) -> &'static mut [PTE] {
        unsafe { core::slice::from_raw_parts_mut(paddr_c(self.0).0 as *mut PTE, PAGE_ITEM_COUNT) }
    }

    #[inline]
    pub const fn get_satp(&self) -> usize {
        (8 << 60) | (self.0 .0 >> 12)
    }

    #[inline]
    pub fn change(&self) {
        unsafe {
            asm!("csrw satp, {0}",  in(reg) self.get_satp());
            // satp::set(Mode::Sv39, 0, self.0.0 >> 12);
            riscv::asm::sfence_vma_all();
        }
    }

    #[inline]
    pub fn map<G>(&self, ppn: PhysPage, vpn: VirtPage, flags: PTEFlags, mut falloc: G, level: usize)
    where
        G: FnMut() -> PhysPage,
    {
        // TODO: Add huge page support.
        let mut page_table = PageTable(self.0);
        for i in (1..level).rev() {
            let value = (vpn.0 >> 9 * i) & 0x1ff;
            let pte = &mut page_table.get_pte_list()[value];
            if i == 0 {
                break;
            }
            if !pte.is_valid() {
                let ppn = falloc();
                *pte = PTE::from_ppn(ppn.0, PTEFlags::V);
            }

            page_table = PageTable(pte.to_ppn().into());
        }

        page_table.get_pte_list()[vpn.0 & 0x1ff] = PTE::from_ppn(ppn.0, flags);
        unsafe {
            sfence_vma(vpn.to_addr(), 0);
        }
    }

    #[inline]
    pub fn unmap(&self, vpn: VirtPage) {
        // TODO: Add huge page support.
        let mut page_table = PageTable(self.0);
        for i in (1..3).rev() {
            let value = (vpn.0 >> 9 * i) & 0x1ff;
            let pte = &mut page_table.get_pte_list()[value];
            if !pte.is_valid() {
                return;
            }
            page_table = PageTable(pte.to_ppn().into());
        }

        page_table.get_pte_list()[vpn.0 & 0x1ff] = PTE::new();
        unsafe {
            sfence_vma(vpn.to_addr(), 0);
        }
    }

    #[inline]
    pub fn virt_to_phys(&self, vaddr: VirtAddr) -> PhysAddr {
        let mut paddr = self.0;
        for i in (0..3).rev() {
            let page_table = PageTable(paddr);
            let value = (vaddr.0 >> 12 + 9 * i) & 0x1ff;
            let pte = &page_table.get_pte_list()[value];
            // 如果当前页是大页 返回相关的位置
            // vaddr.0 % (1 << (12 + 9 * i)) 是大页内偏移
            if pte.is_huge() {
                return PhysAddr(pte.to_ppn().0 << 12 | vaddr.0 % (1 << (12 + 9 * i)));
            }
            paddr = pte.to_ppn().into()
        }
        PhysAddr(paddr.0 | vaddr.0 % PAGE_SIZE)
    }

    #[inline]
    pub fn virt_flags(&self, vaddr: VirtAddr) -> PTEFlags {
        let mut paddr = self.0;
        for i in (0..3).rev() {
            let page_table = PageTable(paddr);
            let value = (vaddr.0 >> 12 + 9 * i) & 0x1ff;
            let pte = &page_table.get_pte_list()[value];
            // 如果当前页是大页 返回相关的位置
            // vaddr.0 % (1 << (12 + 9 * i)) 是大页内偏移
            if pte.is_huge() {
                return pte.flags();
            }
            paddr = pte.to_ppn().into()
        }
        PTEFlags::NONE
    }
}

/// ppn convert, 如果在高半核空间
pub const fn ppn_c(ppn: PhysPage) -> PhysPage {
    PhysPage(ppn.0 | (PAGE_FRAME_BASE >> 12))
}

/// paddr convert, 如果在高半核空间
pub fn paddr_c(paddr: PhysAddr) -> PhysAddr {
    assert!(paddr.0 < PAGE_FRAME_BASE);
    PhysAddr(paddr.0 + PAGE_FRAME_BASE)
}

/// paddr number convert, 如果在高半核空间
pub fn paddr_cn(paddr: usize) -> usize {
    assert!(paddr < PAGE_FRAME_BASE);
    paddr + PAGE_FRAME_BASE
}

/// 虚拟地址转物理地址
pub fn virt_to_phys(vaddr: VirtAddr) -> PhysAddr {
    current_page_table().virt_to_phys(vaddr)
}
