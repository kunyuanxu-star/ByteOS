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

const SD_DRIVER_ADDR: usize = 0xffff_ffc0_0431_0000;
const SOFT_REST_BASE_ADDR: usize = 0xffff_ffc0_0300_3000;
const PINMUX_BASE: usize = 0xffff_ffc0_0300_1000;

const PAD_SDIO0_CD_REG: usize = PINMUX_BASE + 0x34;
const PAD_SDIO0_PWR_EN_REG: usize = PINMUX_BASE + 0x38;
const PAD_SDIO0_CLK_REG: usize = PINMUX_BASE + 0x1C;
const PAD_SDIO0_CMD_REG: usize = PINMUX_BASE + 0x20;
const PAD_SDIO0_D0_REG: usize = PINMUX_BASE + 0x24;
const PAD_SDIO0_D1_REG: usize = PINMUX_BASE + 0x28;
const PAD_SDIO0_D2_REG: usize = PINMUX_BASE + 0x2C;
const PAD_SDIO0_D3_REG: usize = PINMUX_BASE + 0x30;

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
        unsafe { wfi() };
    }
    hexdump(&data);
    println!("读取成功");
}

pub fn init() {
    // println!("read value");
    // let ec_value = unsafe { ((SD_DRIVER_ADDR + 0x28) as *const u32).read() };
    // println!("ec_value: {:#x}", ec_value);
    // let mut ec = HostCtl1PwrBgWup::try_from(ec_value).unwrap();
    // println!("ec: {:#x?} bits: {:#x}", ec, ec_value);
    // ec.lec_ctl().set(u1!(1));
    // unsafe { ((SD_DRIVER_ADDR + 0x28) as *mut u32).write(ec.raw()) };
    // unsafe {
    //     fence_i();
    // }
    // let ec_value = unsafe { ((SD_DRIVER_ADDR + 0x24) as *const u32).read_volatile() };
    // println!("ec_value: {:#x}", ec_value);
    // let ec = PresentState::try_from(ec_value).unwrap();
    // println!("ec: {:#x?} bits: {:#x}", ec, ec_value);
    // unsafe {
    //     fence_i();
    // }
    // let ec_ptr = unsafe {
    //     ((SD_DRIVER_ADDR + 0x24) as *mut PresentState).as_mut().unwrap()
    // };
    // println!("ec: {:#x?} bits: {:#x}", ec_ptr, ec_ptr.raw());

    // let cpu_rest = unsafe {
    //     ((SOFT_REST_BASE_ADDR + 0x24) as *mut SoftCpuRstn).as_mut().unwrap()
    // };
    // println!("cpu_rest: {:#x?} bits: {:#x}", cpu_rest, cpu_rest.raw());
    // cpu_rest.cpucore0().set(u1!(0));
    // cpu_rest.cpucore1().set(u1!(0));
    // cpu_rest.cpucore2().set(u1!(0));
    // cpu_rest.cpucore3().set(u1!(0));
    // cpu_rest.cpusys0().set(u1!(0));
    // cpu_rest.cpusys1().set(u1!(0));
    // cpu_rest.cpusys2().set(u1!(0));
    // let cpu_rest = unsafe {
    //     ((SOFT_REST_BASE_ADDR + 0x24) as *mut SoftCpuRstn).as_mut().unwrap()
    // };
    // println!("cpu_rest: {:#x?} bits: {:#x}", cpu_rest, cpu_rest.raw());

    // let cpu_rest = unsafe {
    //     ((SOFT_REST_BASE_ADDR + 0x20) as *mut SoftCpuacRstn).as_mut().unwrap()
    // };
    // println!("cpu_rest: {:#x?} bits: {:#x}", cpu_rest, cpu_rest.raw());
    // cpu_rest.cpucore0().set(u1!(1));
    // cpu_rest.cpucore1().set(u1!(1));
    // cpu_rest.cpucore2().set(u1!(1));
    // cpu_rest.cpucore3().set(u1!(1));
    // cpu_rest.cpusys0().set(u1!(1));
    // cpu_rest.cpusys1().set(u1!(1));
    // cpu_rest.cpusys2().set(u1!(1));
    // let cpu_rest = unsafe {
    //     ((SOFT_REST_BASE_ADDR + 0x20) as *mut SoftCpuacRstn).as_mut().unwrap()
    // };
    // println!("cpu_rest: {:#x?} bits: {:#x}", cpu_rest, cpu_rest.raw());

    // Initialize sd card gpio

    if check_sd() {
        println!("sdcard exitsts");
        read();
    }
    loop {}
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
