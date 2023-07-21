#![no_std]
#![feature(stdsimd)]
use core::{
    arch::riscv64::{fence_i, wfi},
    fmt::Debug,
};

use bit_struct::*;
use logging::{print, println};

#[macro_use]
extern crate alloc;

const SD_DRIVER_ADDR: usize = 0x0431_0000;
const SOFT_REST_BASE_ADDR: usize = 0x0300_3000;
const PINMUX_BASE: usize = 0x0300_1000;

const PAD_SDIO0_CD_REG: usize = PINMUX_BASE + 0x34;
const PAD_SDIO0_PWR_EN_REG: usize = PINMUX_BASE + 0x38;
const PAD_SDIO0_CLK_REG: usize = PINMUX_BASE + 0x1C;
const PAD_SDIO0_CMD_REG: usize = PINMUX_BASE + 0x20;
const PAD_SDIO0_D0_REG: usize = PINMUX_BASE + 0x24;
const PAD_SDIO0_D1_REG: usize = PINMUX_BASE + 0x28;
const PAD_SDIO0_D2_REG: usize = PINMUX_BASE + 0x2C;
const PAD_SDIO0_D3_REG: usize = PINMUX_BASE + 0x30;

const REG_SDIO0_PAD_MASK: u32 = 0xFFFFFFF3;
const REG_SDIO0_PAD_SHIFT: usize = 2;
const REG_SDIO0_PAD_CLR_MASK: u32 = 0xC;
const REG_SDIO0_CD_PAD_REG: usize = PINMUX_BASE + 0x900;
const REG_SDIO0_CD_PAD_VALUE: u32 = 1;
const REG_SDIO0_CD_PAD_RESET: u32 = 1;
const REG_SDIO0_PWR_EN_PAD_REG: usize = PINMUX_BASE + 0x904;
const REG_SDIO0_PWR_EN_PAD_VALUE: u32 = 2;
const REG_SDIO0_PWR_EN_PAD_RESET: u32 = 2;
const REG_SDIO0_CLK_PAD_REG: usize = PINMUX_BASE + 0xA00;
const REG_SDIO0_CLK_PAD_VALUE: u32 = 2;
const REG_SDIO0_CLK_PAD_RESET: u32 = 2;
const REG_SDIO0_CMD_PAD_REG: usize = PINMUX_BASE + 0xA04;
const REG_SDIO0_CMD_PAD_VALUE: u32 = 1;
const REG_SDIO0_CMD_PAD_RESET: u32 = 2;
const REG_SDIO0_DAT0_PAD_REG: usize = PINMUX_BASE + 0xA08;
const REG_SDIO0_DAT0_PAD_VALUE: u32 = 1;
const REG_SDIO0_DAT0_PAD_RESET: u32 = 2;
const REG_SDIO0_DAT1_PAD_REG: usize = PINMUX_BASE + 0xA0C;
const REG_SDIO0_DAT1_PAD_VALUE: u32 = 1;
const REG_SDIO0_DAT1_PAD_RESET: u32 = 2;
const REG_SDIO0_DAT2_PAD_REG: usize = PINMUX_BASE + 0xA10;
const REG_SDIO0_DAT2_PAD_VALUE: u32 = 1;
const REG_SDIO0_DAT2_PAD_RESET: u32 = 2;
const REG_SDIO0_DAT3_PAD_REG: usize = PINMUX_BASE + 0xA14;
const REG_SDIO0_DAT3_PAD_VALUE: u32 = 1;
const REG_SDIO0_DAT3_PAD_RESET: u32 = 2;


bit_struct! {
    pub struct EmmcCtrl(u32) {
        emmc_func_en: u1,
        latancy_1t: u1,
        clk_free_en: u1,
        disable_data_crc_check: u1,
        reserved: u1,
        emmc_rstn: u1,
        emmc_rstn_oen: u1,
        reserved1: u2,
        cqe_algo_sel: u1,
        cqe_prefetch_disable: u1,
        reserved2: u2,
        timer_clk_sel: u1,
        reserved3: u18
    }

    pub struct HostCtl1PwrBgWup(u32) {
        reserved_27: u5,
        wakeup_on_card_remv: u1,
        wakeup_on_card_insert: u1,
        wakeup_on_card_int: u1,
        reserved_20: u4,
        int_bg: u1,
        read_wait: u1,
        continue_req: u1,
        stop_bg_req: u1,
        reserved_12: u4,
        sd_bus_vol_sel: u3,
        sd_bus_pwr: u1,
        card_det_sel: u1,
        card_det_test: u1,
        ext_dat_width: u1,
        dma_sel: u2,
        hs_enable: u1,
        dat_xfer_width: u1,
        lec_ctl: u1,
    }

    pub struct PresentState(u32) {
        reserved_25: u7,
        cmd_line_state: u1,
        dat_3_0_state: u4,
        card_wp_state: u1,
        card_cd_state: u1,
        card_stable: u1,
        card_inserted: u1,
        reserved_12: u4,
        buf_rd_enable: u1,
        buf_wr_enable: u1,
        rd_xfer_active: u1,
        wr_xfer_active: u1,
        reserved_4: u4,
        re_tune_req: u1,
        dat_line_active: u1,
        cmd_inhibit_dat: u1,
        cmd_inhibit: u1,
    }

    pub struct SoftCpuRstn(u32) {
        reserved: u25,
        cpusys2: u1,
        cpusys1: u1,
        cpusys0: u1,
        cpucore3: u1,
        cpucore2: u1,
        cpucore1: u1,
        cpucore0: u1,
    }

    pub struct SoftCpuacRstn(u32) {
        reserved: u25,
        cpusys2: u1,
        cpusys1: u1,
        cpusys0: u1,
        cpucore3: u1,
        cpucore2: u1,
        cpucore1: u1,
        cpucore0: u1,
    }

    pub struct BlkSizeAndCnt(u32) {
        blk_cnt: u16,
        reserved: u1,
        sdma_buf_bdary: u3,
        xfer_blk_size: u12, // 0x1: 1 byte 0x2: 2 bytes ... 0x200: 512 bytes 0x800: 2048 bytes
    }

    pub struct XferModeAndCmd(u32) {
        reserved_30: u2,
        cmd_idx: u6,
        cmd_type: u2,
        data_present_sel: u1,
        cmd_idx_chk_enable: u1,
        cmd_crc_chk_enable: u1,
        sub_cmd_flag: u1,
        resp_type_sel: u2,
        reserved_9: u7,
        resp_int_enable: u1,
        resp_err_chk_enable: u1,
        resp_type: u1,
        multi_blk_sel: u1,
        dat_xfer_dir: u1,
        auto_cmd_enable: u2,
        blk_cnt_enable: u1,
        dma_enable: u1,
    }

    pub struct NormAndErrIntSts(u32) {
        reserved_29: u3,
        boot_ack_err: u1,
        reserved_27: u1,
        tune_err: u1,
        adma_err: u1,
        auto_cmd_err: u1,
        curr_limit_err: u1,
        dat_endbit_err: u1,
        dat_crc_err: u1,
        dat_tout_err: u1,
        cmd_idx_err: u1,
        cmd_endbit_err: u1,
        cmd_crc_err: u1,
        cmd_tout_err: u1,
        err_int: u1,
        cqe_event: u1,
        reserved_13: u1,
        re_tune_event: u1,
        int_c: u1,
        int_b: u1,
        int_a: u1,
        card_int: u1,
        card_remove_int: u1,
        card_insert_int: u1,
        buf_rrdy: u1,  // Buffer Read Ready
        buf_wrdy: u1,  // Buffer Write Ready
        dma_int: u1,
        bg_event: u1,
        xfer_cmpl: u1, // transfer_complete
        cmd_cmpl: u1,  // command_cmpl
    }

    pub struct ClkCtl(u32) {
        reserved27: u5,
        sw_rst_dat: u1,
        sw_rst_cmd: u1,
        sw_rst_all: u1,
        reserved20: u4,
        tout_cnt: u4,
        freq_sel: u8,
        up_freq_sel: u2,
        reserved4: u2,
        pll_en: u1,
        sd_clk_en: u1,
        int_clk_stable: u1,
        int_clk_en: u1,
    }

    pub struct AutoCmdErrAndHostCtl2(u32) {
        present_val_enable: u1,
        async_int_en: u1,
        reserved_24: u6,
        sample_clk_sel: u1,
        execute_time: u1,
        drv_sel: u2,
        en_18_sig: u1,
        uhs_mode_sel: u3,
        reserved_8: u8,
        cmd_not_issue_by_cmd12: u1,
        reserved_5: u2,
        auto_cmd_idx_err: u1,
        auto_cmd_endbit_err: u1,
        auto_cmd_crc_err: u1,
        auto_cmd_tout_err: u1,
        auto_cmd12_no_exe: u1,
    }

    pub struct Capabilities1(u32) {
        slot_type: u2,
        async_int_support: u1,
        bus64_support: u1,
        reserved_27: u1,
        v18_support: u1,
        v30_support: u1,
        v33_support: u1,
        susp_res_support: u1,
        sdma_support: u1,
        hs_support: u1,
        reserved_20: u1,
        adma2_support: u1,
        embedded_8bit: u1,
        max_blk_len: u2,
        base_clk_freq: u8,
        tout_clk_unit: u1,
        reserved_6: u1,
        tout_clk_freq: u6,
    }

    pub struct Capabilities2(u32) {
        reserved_24: u8,
        clk_multiplier: u8,
        retune_mode: u2,
        tune_sdr50: u1,
        reserved_12: u1,
        retune_timer: u4,
        reserved_7: u1,
        drv_d_support: u1,
        drv_c_support: u1,
        drv_a_support: u1,
        reserved_3: u1,
        ddr50_support: u1,
        sdr104_support: u1,
        sdr50_support: u1,
    }
}

pub fn reg_transfer<T>(offset: usize) -> &'static mut T {
    unsafe { ((SD_DRIVER_ADDR + offset) as *mut T).as_mut().unwrap() }
}

/// check the sdcard that was inserted
pub fn check_sd() -> bool {
    let present_state = reg_transfer::<PresentState>(0x24);
    present_state.card_inserted().get() == u1!(1)
}

// 1. 写入 BLK_SIZE 暂存器来设定块大小
// 2. 写入 BLK_CNT 暂存器来设定块数量
// 3. 写入 ARGUMENT 暂存器来设定指令参数
// 4. 写入 XFER_ARGUMENT 暂存器来设定传输模式.
// 5. 写入 CMD 暂存器来设定指令和相应的类型
// 6. 等待 Command 完成的中断 NORM_INT_STS[CMD_CMPL]
// 7. 收到中断后设定 NORM_INT_STS[CMD_CMPL]=1来清除CMD_CMPL中断状态
// 8. 接着读取 RESP1_0, RESP3_2, RESP5_4, RESP7_6 暂存器来获取响应值

// 读
// 9. 等待 Buffer Read Ready 的中断 NORM_INT_STS[BUF_RRDY]
// 10. 收到中断后设定 NORM_INT_STS[BUF_RRDY]=1 来清除 BUFF_RRDY 中断状态
// 11. 依序从 BUF_DATA 暂存器读取从装置端接收回来的资料
// 12. 如果还有更多的 block 需要读取，则回到步骤 9，直到最后一个 block 读取完

// 写
// 9. 等待 Buffer Write Ready 的中断 NORM_INT_STS[BUF_WRDY]
// 10. 收到中断后设定 NORM_INT_STS[BUF_WRDY]=1 来清除 BUFF_WRDY 中断状态
// 11. 将想要写入装置的数据依序写入 BUF_DATA 暂存器
// 12. 如果还有更多的 block 需要写入，则回到步骤 9，直到最后一个 block 写完

// 13. 判断是单一模块传输、多模块传输或者无限模块传输。如果是单模块传输或者多模块传输则继续执行，如果是无限模块传输则执行 16
// 14. 等待数据传输完成的中断 NORM_INT_STS[XFER_CMPL]
// 15. 收到中断后设定 NORM_INT_STS[CMD_XFER]=1 来清除 CMD_XFER 中断状态
// 16. 执行中止中断传输程序

pub fn read() {
    println!("开始读取");
    let mut data = vec![0u8; 512];

    let blk_size_and_cnt = reg_transfer::<BlkSizeAndCnt>(0x4);
    blk_size_and_cnt.xfer_blk_size().set(u12!(0x200));
    blk_size_and_cnt.blk_cnt().set(1);

    // todo: write cmd to  argument reg
    let argument = reg_transfer::<u32>(0x8);
    *argument = 0;

    let xfer_mode = reg_transfer::<XferModeAndCmd>(0xc);
    xfer_mode.dma_enable().set(u1!(0));
    xfer_mode.blk_cnt_enable().set(u1!(0)); // just transfer 1 block
    xfer_mode.dat_xfer_dir().set(u1!(1)); // 1. read 0. write
    xfer_mode.multi_blk_sel().set(u1!(0)); // 0. single block   1. multi blocks
                                           // xfer_mode.cmd_idx().set(u6!(52));    // write command 52 for writing/reading.
    xfer_mode.cmd_idx().set(u6!(17)); // 17: single read  18: multi read
                                      // 24: single write 25: multi write

    let norm_int_sts = reg_transfer::<NormAndErrIntSts>(0x30);
    logging::println!("{:#x?}", norm_int_sts);
    loop {
        if norm_int_sts.cmd_cmpl().get() == u1!(1) {
            norm_int_sts.cmd_cmpl().set(u1!(1));
            break;
        }
        // unsafe { wfi() };
    }

    let resp1_0 = reg_transfer::<u32>(0x10);
    let resp3_2 = reg_transfer::<u32>(0x14);
    let resp5_4 = reg_transfer::<u32>(0x18);
    let resp7_6 = reg_transfer::<u32>(0x1c);

    println!(
        "resp: {:#x} {:#x} {:#x} {:#x}",
        resp1_0, resp3_2, resp5_4, resp7_6
    );

    loop {
        if norm_int_sts.buf_rrdy().get() == u1!(0) {
            norm_int_sts.buf_rrdy().set(u1!(1));
            break;
        }
        unsafe { wfi() };
    }

    for i in 0..512 {
        let buf_data_ptr = (SD_DRIVER_ADDR + 0x20) as *mut u32;
        data[i] = unsafe { buf_data_ptr.read_volatile() } as _;
    }

    loop {
        if norm_int_sts.xfer_cmpl().get() == u1!(0) {
            norm_int_sts.xfer_cmpl().set(u1!(1));
            break;
        }
        // unsafe { wfi() };
    }
    hexdump(&data);
    println!("读取成功");
}

pub fn mmio_clrsetbits_32(addr: *mut u32, clear: u32, set: u32) {
    unsafe {
        *addr = (*addr & !clear) | set;
    }
}

pub fn mmio_clearbits_32(addr: *mut u32, clear: u32) {
    unsafe {
        *addr = *addr & !clear;
    }
}

pub fn mmio_setbits_32(addr: *mut u32, set: u32) {
    unsafe {
        *addr = *addr| set;
    }
}

pub fn mmio_write_32(addr: *mut u32, value: u32) {
    unsafe {
        *addr = value;
    }
}

pub fn mmio_read_32(addr: *mut u32) -> u32 {
    unsafe {
        *addr
    }
}

pub fn init() {
    // Initialize sd card gpio
    if check_sd() {
        println!("sdcard exitsts");
        // try to change voltage
        // unsafe {
        //     let sd_pwrsw_ctl_ptr = 0x0300_01F4 as *mut u32;
        //     // bit 0 reg_en_pwrsw 1
        //     // bit 1 reg_pwrsw_vsel 1 (1: 1.8v 0: 3.3v)
        //     // bit 2 reg_pwrsw_disc 0
        //     // bit 3 reg_pwrsw_auto 1
        //     *sd_pwrsw_ctl_ptr = 0b1011;
        //     for _ in 0..0x100_0000 {}
        // }
        // try to reset sd
        unsafe {
            let reset_ptr = 0x0300_3000 as *mut u32;
            // bit 15, emmc_rst, bit 16, sdio0_rst, bit 17, sdio1_rst
            *reset_ptr |= 1 << 16;
            for _ in 0..0x100_0000 {}
        }
        // try to set pinmux
        unsafe {
            // sdhci_probe
            // let SDHCI_HOST_VERSION = (SD_DRIVER_ADDR + 0xfe) as *mut u32;
            // println!("spec_ver: {}, number: {}", *SDHCI_HOST_VERSION & 0xf, (*SDHCI_HOST_VERSION >> 8) & 0xf);
            // sdhci_reset
            // mmio_clearbits_32((SD_DRIVER_ADDR + 0x2c) as _, 1 << 24);
            let rst = (SD_DRIVER_ADDR + 0x2f) as *mut u8;
            loop {
                if *rst & 0x1 == 0x0 {
                    break;
                }
                for _ in 0..0x100_0000 {}
            }

            // cvi_general_reset
            let SDHCI_HOST_CONTROL2 = (SD_DRIVER_ADDR + 0x3e) as *mut u16;
            println!("sdcard ctl2: {:#x}", *SDHCI_HOST_CONTROL2);
            mmio_setbits_32((SD_DRIVER_ADDR + 0x200) as _, 1 << 1);
            // mmio_clearbits_32((SD_DRIVER_ADDR + 0x200) as _, 1 << 1);
            // mmio_setbits_32((SD_DRIVER_ADDR + 0x24c) as _, 1 << 0);
            //reg_0x240[25:24] = 1 reg_0x240[22:16] = 0 reg_0x240[9:8] = 1 reg_0x240[6:0] = 0
            // mmio_clrsetbits_32((SD_DRIVER_ADDR + 0x240) as _, (0b111_1111 << 16) | (0b111_1111 << 0), (3 << 8) | (3 << 24));
            

            // let TOP_BASE: usize = 0x03000000;
            // let REG_TOP_SD_PWRSW_CTRL: usize = 0x1F4;
            // mmio_write_32((TOP_BASE + REG_TOP_SD_PWRSW_CTRL) as _, 0x9);

            // // let val: u8 = (bunplug) ? 0x3 : 0x0;
            // let reset = false;
            // let val = if reset {
            //     0x3
            // } else {
            //     0x0
            // };

            // mmio_write_32(PAD_SDIO0_CD_REG as _, 0x0);
            // mmio_write_32(PAD_SDIO0_PWR_EN_REG as _, 0x0);
            // mmio_write_32(PAD_SDIO0_CLK_REG as _, val as _);
            // mmio_write_32(PAD_SDIO0_CMD_REG as _, val as _);
            // mmio_write_32(PAD_SDIO0_D0_REG as _, val as _);
            // mmio_write_32(PAD_SDIO0_D1_REG as _, val as _);
            // mmio_write_32(PAD_SDIO0_D2_REG as _, val as _);
            // mmio_write_32(PAD_SDIO0_D3_REG as _, val as _);

            // if reset {
            //     mmio_clrsetbits_32(REG_SDIO0_PWR_EN_PAD_REG as _, REG_SDIO0_PAD_CLR_MASK,
            //         REG_SDIO0_PWR_EN_PAD_RESET << REG_SDIO0_PAD_SHIFT);
 
            //     mmio_clrsetbits_32(REG_SDIO0_CD_PAD_REG as _, REG_SDIO0_PAD_CLR_MASK,
            //                 REG_SDIO0_CD_PAD_RESET << REG_SDIO0_PAD_SHIFT);
        
            //     mmio_clrsetbits_32(REG_SDIO0_CLK_PAD_REG as _, REG_SDIO0_PAD_CLR_MASK,
            //                 REG_SDIO0_CLK_PAD_RESET << REG_SDIO0_PAD_SHIFT);
        
            //     mmio_clrsetbits_32(REG_SDIO0_CMD_PAD_REG as _, REG_SDIO0_PAD_CLR_MASK,
            //                 REG_SDIO0_CMD_PAD_RESET << REG_SDIO0_PAD_SHIFT);
        
            //     mmio_clrsetbits_32(REG_SDIO0_DAT1_PAD_REG as _, REG_SDIO0_PAD_CLR_MASK,
            //                 REG_SDIO0_DAT1_PAD_RESET << REG_SDIO0_PAD_SHIFT);
        
            //     mmio_clrsetbits_32(REG_SDIO0_DAT0_PAD_REG as _, REG_SDIO0_PAD_CLR_MASK,
            //                 REG_SDIO0_DAT0_PAD_RESET << REG_SDIO0_PAD_SHIFT);
        
            //     mmio_clrsetbits_32(REG_SDIO0_DAT2_PAD_REG as _, REG_SDIO0_PAD_CLR_MASK,
            //                 REG_SDIO0_DAT2_PAD_RESET << REG_SDIO0_PAD_SHIFT);
        
            //     mmio_clrsetbits_32(REG_SDIO0_DAT3_PAD_REG as _, REG_SDIO0_PAD_CLR_MASK,
            //                 REG_SDIO0_DAT3_PAD_RESET << REG_SDIO0_PAD_SHIFT);
            // } else {
            //     mmio_clrsetbits_32(REG_SDIO0_PWR_EN_PAD_REG as _, REG_SDIO0_PAD_CLR_MASK,
            //         REG_SDIO0_PWR_EN_PAD_VALUE << REG_SDIO0_PAD_SHIFT);
    
            //     mmio_clrsetbits_32(REG_SDIO0_CD_PAD_REG as _, REG_SDIO0_PAD_CLR_MASK,
            //                 REG_SDIO0_CD_PAD_VALUE << REG_SDIO0_PAD_SHIFT);
    
            //     mmio_clrsetbits_32(REG_SDIO0_CLK_PAD_REG as _, REG_SDIO0_PAD_CLR_MASK,
            //                 REG_SDIO0_CLK_PAD_VALUE << REG_SDIO0_PAD_SHIFT);
    
            //     mmio_clrsetbits_32(REG_SDIO0_CMD_PAD_REG as _, REG_SDIO0_PAD_CLR_MASK,
            //                 REG_SDIO0_CMD_PAD_VALUE << REG_SDIO0_PAD_SHIFT);
    
            //     mmio_clrsetbits_32(REG_SDIO0_DAT1_PAD_REG as _, REG_SDIO0_PAD_CLR_MASK,
            //                 REG_SDIO0_DAT1_PAD_VALUE << REG_SDIO0_PAD_SHIFT);
    
            //     mmio_clrsetbits_32(REG_SDIO0_DAT0_PAD_REG as _, REG_SDIO0_PAD_CLR_MASK,
            //                 REG_SDIO0_DAT0_PAD_VALUE << REG_SDIO0_PAD_SHIFT);
    
            //     mmio_clrsetbits_32(REG_SDIO0_DAT2_PAD_REG as _, REG_SDIO0_PAD_CLR_MASK,
            //                 REG_SDIO0_DAT2_PAD_VALUE << REG_SDIO0_PAD_SHIFT);
    
            //     mmio_clrsetbits_32(REG_SDIO0_DAT3_PAD_REG as _, REG_SDIO0_PAD_CLR_MASK,
            //                 REG_SDIO0_DAT3_PAD_VALUE << REG_SDIO0_PAD_SHIFT);
            // }
            // // reset sdcard
            // mmio_clearbits_32((SD_DRIVER_ADDR + 0x2c) as *mut u32, (1 << 24) | (1 << 25) | (1 << 26));
            // mmio_setbits_32(SD_DRIVER_ADDR + 0x3c, );
            let host_pwr = reg_transfer::<HostCtl1PwrBgWup>(0x28);
            println!("{:#x?}", host_pwr);
        }
        // get sd pll divider.
        unsafe {
            // open sd0 clock
            let clk_en0 = 0x0300_2000 as *mut u32;
            println!("all_status: {:#x}, clk_sd0_en: {}, clk_100k_sd0_en: {}", *clk_en0, (*clk_en0 >> 19) & 1, (*clk_en0 >> 20) & 1);
            let div_clk_sd0 = (0x0300_2000 + 0x78) as *mut u32;
            println!("div_clk_sd0: {:#x}", *div_clk_sd0);
        }
        // get sd support info
        unsafe {
            // let supp = (SD_DRIVER_ADDR + 0x44) as *mut u8;
            // println!("support: {:#x}", *supp);
            let cap1 = reg_transfer::<Capabilities1>(0x40);
            println!("{:#x?}", cap1);
            let cap2 = reg_transfer::<Capabilities2>(0x44);
            println!("{:#x?}", cap2);
        }
        // try to get host mode
        unsafe {
            let host_ctl2 = reg_transfer::<AutoCmdErrAndHostCtl2>(0x3c);
            host_ctl2.uhs_mode_sel().set(u3!(3));
            println!("{:#x?}", *host_ctl2);
        }
        // try to shutdown sdio clock.
        unsafe {
            let present_state = reg_transfer::<PresentState>(0x24);
            println!("present_state: {:#x?}", present_state);
            if present_state.cmd_inhibit().get() == u1!(0) && present_state.dat_line_active().get() == u1!(0) {
                println!("CLK_CTL[SD_CLK_EN]=0 Close sdio clock");
                reg_transfer::<ClkCtl>(0x2c).sd_clk_en().set(u1!(0));
            }
            for _ in 0..0x100_0000 {}
        }
        // // try to set clock.
        // unsafe {
        //     let clk_ctl = reg_transfer::<ClkCtl>(0x2c);
        //     println!("present_state: {:#x?}", clk_ctl);
        //     clk_ctl.sd_clk_en().set(u1!(0));
        //     // set clock freq, out = internal_clock_freq / (2 x freq_sel)
        //     clk_ctl.freq_sel().set(1);
        //     clk_ctl.int_clk_en().set(u1!(1));
        //     println!("present_state: {:#x?}", clk_ctl);
        //     loop {
        //         println!("present_state: {:#x?}", clk_ctl);
        //         if clk_ctl.int_clk_stable().get() == u1!(1) {
        //             break;
        //         }
        //         for _ in 0..0x100_0000 {}
        //     }
        //     clk_ctl.sd_clk_en().set(u1!(1));
        //     for _ in 0..0x100_0000 {}
        // }
        read();
    }
    panic!("manual shutdown @ cv1811-sd");
    // loop {}
    // let ec = EmmcCtrl::new(u1!(1));
    // println!("ec: {:#x?}  bits: {:#x}", ec, ec.raw());
}

#[no_mangle]
pub fn hexdump(data: &[u8]) {
    const PRELAND_WIDTH: usize = 70;
    println!("{:-^1$}", " hexdump ", PRELAND_WIDTH);
    for offset in (0..data.len()).step_by(16) {
        for i in 0..16 {
            if offset + i < data.len() {
                print!("{:02x} ", data[offset + i]);
            } else {
                print!("{:02} ", "");
            }
        }

        print!("{:>6}", ' ');

        for i in 0..16 {
            if offset + i < data.len() {
                let c = data[offset + i];
                if c >= 0x20 && c <= 0x7e {
                    print!("{}", c as char);
                } else {
                    print!(".");
                }
            } else {
                print!("{:02} ", "");
            }
        }

        println!("");
    }
    println!("{:-^1$}", " hexdump end ", PRELAND_WIDTH);
}
