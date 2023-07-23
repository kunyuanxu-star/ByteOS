#![no_std]
#![feature(stdsimd)]

mod consts;
mod utils;

use core::arch::asm;

use bit_struct::*;
use consts::*;
use frame_allocator::frame_alloc;
use log::debug;
use logging::{print, println};
use utils::{mmio_clearbits_32, mmio_clrsetbits_32, mmio_setbits_32, reg_transfer};

use crate::{
    consts::{HostCtl1PwrBgWup, PresentState, REG_SDIO0_CLK_PAD_REG},
    utils::{check_sd, mmio_read_32, mmio_write_32, test::hexdump},
};

#[macro_use]
extern crate alloc;

/// read a block from the sdcard.
pub fn read_block(block_id: u32, data: &mut [u8]) -> Result<(), CmdError> {
    cmd_transfer(CommandType::CMD(17), block_id, 1);
    read_buff(data)?;
    let res = wait_for_xfer_done();
    mmio_write_32(
        (SD_DRIVER_ADDR + 0x30) as _,
        mmio_read_32((SD_DRIVER_ADDR + 0x30) as _),
    );
    res
}

pub fn read_scd(rsa: u32) -> Result<(), CmdError> {
    log::debug!("开始读取 scd");
    let mut data = vec![0u8; 512];

    cmd_transfer(CommandType::CMD(55), rsa, 0);
    cmd_transfer(CommandType::ACMD(51), 0, 1);

    read_buff(&mut data)?;

    wait_for_xfer_done()?;
    mmio_write_32(
        (SD_DRIVER_ADDR + 0x30) as _,
        mmio_read_32((SD_DRIVER_ADDR + 0x30) as _),
    );
    hexdump(&data);
    log::debug!("读取成功");
    Ok(())
}

pub fn reset_config() {
    unsafe {
        // disable power
        // NOTE: This will close the bus power, but i don't how to restart again.
        power_config(PowerLevel::Close);

        // reset
        mmio_clearbits_32(
            (SD_DRIVER_ADDR + 0x2c) as *mut u32,
            (1 << 24) | (1 << 25) | (1 << 26),
        );
        for _ in 0..0x100_0000 {
            asm!("nop")
        }
        // enable power
        power_config(PowerLevel::V33);

        // high_speed and data width 4 bit
        // mmio_setbits_32((SD_DRIVER_ADDR + 0x28) as _, (1 << 1) | (1 << 2));
        mmio_setbits_32((SD_DRIVER_ADDR + 0x28) as _, 1 << 2);

        // *((SD_DRIVER_ADDR + 0x28) as *mut u8) |= 1 << 3;
    }
}

pub fn wait_for_cmd_done() -> Result<(), CmdError> {
    let norm_int_sts = reg_transfer::<NormAndErrIntSts>(0x30);
    loop {
        if norm_int_sts.err_int().get() == true {
            println!("{:#x?}", norm_int_sts);
            mmio_write_32((SD_DRIVER_ADDR + 0x30) as _, 1 << 15);
            break Err(CmdError::IntError);
        }
        if norm_int_sts.cmd_cmpl().get() == true {
            println!("{:#x?}", norm_int_sts);
            // norm_int_sts.cmd_cmpl().set(true);
            mmio_write_32((SD_DRIVER_ADDR + 0x30) as _, 1 << 0);
            break Ok(());
        }
        for _ in 0..1000 {
            unsafe { asm!("nop") }
        }
    }
}

pub fn wait_for_xfer_done() -> Result<(), CmdError> {
    let norm_int_sts = reg_transfer::<NormAndErrIntSts>(0x30);
    loop {
        if norm_int_sts.xfer_cmpl().get() == true {
            mmio_write_32((SD_DRIVER_ADDR + 0x30) as _, 1 << 1);
            break Ok(());
        }
        if norm_int_sts.err_int().get() == true {
            mmio_write_32((SD_DRIVER_ADDR + 0x30) as _, 1 << 15);
            break Err(CmdError::IntError);
        }
        for _ in 0..1000 {
            unsafe { asm!("nop") }
        }
    }
}

/*
04310000: 00000001 00007200 00000000 113a0013  .....r........:.
04310010: 00000900 00edc87f 32db7900 00400e00  .........y.2..@.
04310020: 00004000 03f70000 00000f06 000e0107  .@..............
04310030: 00000000 027f003b 00000000 30880000  ....;..........0
04310040: 3f68c832 08006077 00000000 00000000  2.h?w`..........
04310050: 00000000 00000000 82dc0d40 00000000  ........@.......
04310060: 00030003 00030002 00010002 00020000  ................


second
04310000: 00000001 00007200 00000001 113a0013  .....r........:.
04310010: 00000900 00edc87f 32db7900 00400e00  .........y.2..@.
04310020: 00004000 03f70000 00000f06 000e0407  .@..............
04310030: 00000000 027f003b 00000000 30830000  ....;..........0
04310040: 3f68c832 08006077 00000000 00000000  2.h?w`..........
04310050: 00000000 00000000 80100200 00000000  ................
04310060: 00030003 00030002 00010002 00020000  ................
*/

pub fn cmd_transfer(cmd: CommandType, arg: u32, blk_cnt: u32) -> Result<(), CmdError> {
    let present_state = reg_transfer::<PresentState>(0x24);

    while present_state.cmd_inhibit_dat().get() == true
        || present_state.cmd_inhibit_dat().get() == true
    {}

    let mut flags: u32 = (cmd.num() as u32) << 24;

    const BLK_CNT_EN: u32 = 1 << 1;
    const XFER_READ: u32 = 1 << 4;
    const DATA_PRESENT: u32 = 1 << 21;
    const L48: u32 = 2 << 16;
    const L48_BUSY: u32 = 2 << 16;
    const L136: u32 = 1 << 16;
    const CRC_CHECK_EN: u32 = 1 << 19;
    const INX_CHECK_EN: u32 = 1 << 20;

    if blk_cnt > 0 {
        // set blk size and blk count
        mmio_write_32((SD_DRIVER_ADDR + 0x04) as _, 0x200 | (blk_cnt << 16));
        flags |= BLK_CNT_EN;
    }

    flags |= match cmd {
        CommandType::CMD(17) => DATA_PRESENT | XFER_READ,
        CommandType::ACMD(51) => DATA_PRESENT | XFER_READ,
        _ => 0,
    };

    flags |= match cmd {
        // R1
        CommandType::ACMD(6)
        | CommandType::ACMD(42)
        | CommandType::ACMD(51)
        | CommandType::CMD(17)
        | CommandType::CMD(8)
        | CommandType::CMD(16)
        | CommandType::CMD(7) => L48 | CRC_CHECK_EN | INX_CHECK_EN,
        // R2
        CommandType::CMD(2) | CommandType::CMD(9) => L136 | CRC_CHECK_EN,
        // R3
        CommandType::ACMD(41) | CommandType::CMD(58) => L48,
        // R6
        CommandType::CMD(3) => L48_BUSY | CRC_CHECK_EN | INX_CHECK_EN,
        _ => 0,
    };

    // flags |= 0x10;

    unsafe {
        // set blk cnt
        *((SD_DRIVER_ADDR + 0x06) as *mut u8) = 0;
        // set timeout time
        *((SD_DRIVER_ADDR + 0x2e) as *mut u8) = 0xe;
        *((SD_DRIVER_ADDR + 0x30) as *mut u32) = 0xF3FFFFFF;
        *((SD_DRIVER_ADDR + 0x8) as *mut u32) = arg;
        *((SD_DRIVER_ADDR + 0xc) as *mut u32) = flags;
    }

    wait_for_cmd_done()?;

    let resp1_0 = reg_transfer::<u32>(0x10);
    let resp3_2 = reg_transfer::<u32>(0x14);
    let resp5_4 = reg_transfer::<u32>(0x18);
    let resp7_6 = reg_transfer::<u32>(0x1c);

    println!(
        "resp: {:#x} {:#x} {:#x} {:#x}",
        resp1_0, resp3_2, resp5_4, resp7_6
    );

    Ok(())
}

pub fn dma_read() {
    println!(
        "start dma read ctl: {:#x?}",
        *reg_transfer::<HostCtl1PwrBgWup>(0x28)
    );
    let ppn = frame_alloc().expect("can't alloc memory");
    mmio_write_32((SD_DRIVER_ADDR + 0x0) as _, ppn.0.to_addr() as _);

    // set blk size and blk count
    // let blk_size_and_cnt = reg_transfer::<BlkSizeAndCnt>(0x4);
    // blk_size_and_cnt.xfer_blk_size().set(u12!(0x200));
    // blk_size_and_cnt.blk_cnt().set(1);
    mmio_write_32(
        (SD_DRIVER_ADDR + 0x4) as _,
        (0x200 << 0) | (0 << 12) | (1 << 16),
    );

    // todo: write cmd to argument reg
    *reg_transfer::<u32>(0x8) = 1;

    const DMA_EN: u32 = 1 << 0;
    const BLK_CNT_EN: u32 = 1 << 1;
    const AUTO_CMD_EN: u32 = 0 << 2;
    const DAT_XFER_READ: u32 = 1 << 4;
    const RESP_R1: u32 = 0 << 6;
    const L48: u32 = 2 << 16;
    const CMD_CRC_CHK_EN: u32 = 1 << 19;
    const CMD_IDX_CHK_EN: u32 = 1 << 20;
    const DATA_PRESENT: u32 = 1 << 21;

    unsafe {
        // set blk cnt
        *((SD_DRIVER_ADDR + 0x06) as *mut u8) = 1;
        // set timeout time
        *((SD_DRIVER_ADDR + 0x2e) as *mut u8) = 0xe;
        let cmd = DMA_EN
            | BLK_CNT_EN
            | DAT_XFER_READ
            | RESP_R1
            | L48
            | CMD_CRC_CHK_EN
            | CMD_IDX_CHK_EN
            | DATA_PRESENT
            | (17 << 24);
        // assert_eq!(cmd, 0x113A0013);
        *((SD_DRIVER_ADDR + 0xc) as *mut u32) = cmd;
    }

    println!("dma_read_req sended");
    match wait_for_cmd_done() {
        Ok(_) => {}
        Err(_) => {
            println!("time out for dma read");
        }
    }

    let resp1_0 = reg_transfer::<u32>(0x10);
    let resp3_2 = reg_transfer::<u32>(0x14);
    let resp5_4 = reg_transfer::<u32>(0x18);
    let resp7_6 = reg_transfer::<u32>(0x1c);

    println!(
        "resp: {:#x} {:#x} {:#x} {:#x}",
        resp1_0, resp3_2, resp5_4, resp7_6
    );

    let norm_int_sts = reg_transfer::<NormAndErrIntSts>(0x30);

    loop {
        logging::println!(
            "{:#x?}  present_sts: {:#x?}",
            norm_int_sts,
            *reg_transfer::<PresentState>(0x24)
        );
        if norm_int_sts.xfer_cmpl().get() {
            println!("transfer done");
            break;
        }
        // if norm_int_sts.buf_rrdy().get() {
        //     mmio_write_32((SD_DRIVER_ADDR + 0x30) as _, 1 << 5);
        //     break;
        // }
        if norm_int_sts.dma_int().get() {
            // norm_int_sts.dma_int().set(true);
            mmio_write_32((SD_DRIVER_ADDR + 0x30) as _, 1 << 3);
            println!("transfer done");
            break;
        }
        for _ in 0..0x1000_0000 {
            unsafe { asm!("nop") }
        }
    }

    norm_int_sts.xfer_cmpl().set(true);
    norm_int_sts.dma_int().set(true);

    hexdump(unsafe {
        core::slice::from_raw_parts_mut((ppn.0.to_addr() | 0xffff_ffc0_0000_0000) as *mut u8, 512)
    });
}

pub fn pad_settings() {
    mmio_write_32((TOP_BASE + REG_TOP_SD_PWRSW_CTRL) as _, 0x9);

    // let val: u8 = (bunplug) ? 0x3 : 0x0;
    let reset = false;

    let val = if reset { 0x3 } else { 0x0 };

    mmio_write_32(PAD_SDIO0_CD_REG as _, 0x0);
    mmio_write_32(PAD_SDIO0_PWR_EN_REG as _, 0x0);
    mmio_write_32(PAD_SDIO0_CLK_REG as _, val as _);
    mmio_write_32(PAD_SDIO0_CMD_REG as _, val as _);
    mmio_write_32(PAD_SDIO0_D0_REG as _, val as _);
    mmio_write_32(PAD_SDIO0_D1_REG as _, val as _);
    mmio_write_32(PAD_SDIO0_D2_REG as _, val as _);
    mmio_write_32(PAD_SDIO0_D3_REG as _, val as _);

    if reset {
        mmio_clrsetbits_32(
            REG_SDIO0_PWR_EN_PAD_REG as _,
            REG_SDIO0_PAD_CLR_MASK,
            REG_SDIO0_PWR_EN_PAD_RESET << REG_SDIO0_PAD_SHIFT,
        );

        mmio_clrsetbits_32(
            REG_SDIO0_CD_PAD_REG as _,
            REG_SDIO0_PAD_CLR_MASK,
            REG_SDIO0_CD_PAD_RESET << REG_SDIO0_PAD_SHIFT,
        );

        mmio_clrsetbits_32(
            REG_SDIO0_CLK_PAD_REG as _,
            REG_SDIO0_PAD_CLR_MASK,
            REG_SDIO0_CLK_PAD_RESET << REG_SDIO0_PAD_SHIFT,
        );

        mmio_clrsetbits_32(
            REG_SDIO0_CMD_PAD_REG as _,
            REG_SDIO0_PAD_CLR_MASK,
            REG_SDIO0_CMD_PAD_RESET << REG_SDIO0_PAD_SHIFT,
        );

        mmio_clrsetbits_32(
            REG_SDIO0_DAT1_PAD_REG as _,
            REG_SDIO0_PAD_CLR_MASK,
            REG_SDIO0_DAT1_PAD_RESET << REG_SDIO0_PAD_SHIFT,
        );

        mmio_clrsetbits_32(
            REG_SDIO0_DAT0_PAD_REG as _,
            REG_SDIO0_PAD_CLR_MASK,
            REG_SDIO0_DAT0_PAD_RESET << REG_SDIO0_PAD_SHIFT,
        );

        mmio_clrsetbits_32(
            REG_SDIO0_DAT2_PAD_REG as _,
            REG_SDIO0_PAD_CLR_MASK,
            REG_SDIO0_DAT2_PAD_RESET << REG_SDIO0_PAD_SHIFT,
        );

        mmio_clrsetbits_32(
            REG_SDIO0_DAT3_PAD_REG as _,
            REG_SDIO0_PAD_CLR_MASK,
            REG_SDIO0_DAT3_PAD_RESET << REG_SDIO0_PAD_SHIFT,
        );
    } else {
        mmio_clrsetbits_32(
            REG_SDIO0_PWR_EN_PAD_REG as _,
            REG_SDIO0_PAD_CLR_MASK,
            REG_SDIO0_PWR_EN_PAD_VALUE << REG_SDIO0_PAD_SHIFT,
        );

        mmio_clrsetbits_32(
            REG_SDIO0_CD_PAD_REG as _,
            REG_SDIO0_PAD_CLR_MASK,
            REG_SDIO0_CD_PAD_VALUE << REG_SDIO0_PAD_SHIFT,
        );

        mmio_clrsetbits_32(
            REG_SDIO0_CLK_PAD_REG as _,
            REG_SDIO0_PAD_CLR_MASK,
            REG_SDIO0_CLK_PAD_VALUE << REG_SDIO0_PAD_SHIFT,
        );

        mmio_clrsetbits_32(
            REG_SDIO0_CMD_PAD_REG as _,
            REG_SDIO0_PAD_CLR_MASK,
            REG_SDIO0_CMD_PAD_VALUE << REG_SDIO0_PAD_SHIFT,
        );

        mmio_clrsetbits_32(
            REG_SDIO0_DAT1_PAD_REG as _,
            REG_SDIO0_PAD_CLR_MASK,
            REG_SDIO0_DAT1_PAD_VALUE << REG_SDIO0_PAD_SHIFT,
        );

        mmio_clrsetbits_32(
            REG_SDIO0_DAT0_PAD_REG as _,
            REG_SDIO0_PAD_CLR_MASK,
            REG_SDIO0_DAT0_PAD_VALUE << REG_SDIO0_PAD_SHIFT,
        );

        mmio_clrsetbits_32(
            REG_SDIO0_DAT2_PAD_REG as _,
            REG_SDIO0_PAD_CLR_MASK,
            REG_SDIO0_DAT2_PAD_VALUE << REG_SDIO0_PAD_SHIFT,
        );

        mmio_clrsetbits_32(
            REG_SDIO0_DAT3_PAD_REG as _,
            REG_SDIO0_PAD_CLR_MASK,
            REG_SDIO0_DAT3_PAD_VALUE << REG_SDIO0_PAD_SHIFT,
        );
    }
}

/// get sdcard support info
pub fn get_sd_caps() {
    // let supp = (SD_DRIVER_ADDR + 0x44) as *mut u8;
    // println!("support: {:#x}", *supp);
    let cap1 = reg_transfer::<Capabilities1>(0x40);
    println!("{:#x?}", cap1);
    let cap2 = reg_transfer::<Capabilities2>(0x44);
    println!("{:#x?}", cap2);
    // power on
    println!("{:#x?}", *reg_transfer::<HostCtl1PwrBgWup>(0x28))
}

#[derive(PartialEq)]
pub enum PowerLevel {
    V33,
    V30,
    V18,
    Close,
}

pub fn power_config(level: PowerLevel) {
    const SD_BUS_VSEL_3V3_MASK: u8 = 0b111 << 1;
    const SD_BUS_VSEL_3V0_MASK: u8 = 0b110 << 1;
    const SD_BUS_VSEL_1V8_MASK: u8 = 0b101 << 1;
    const SD_BUS_PWR_MASK: u8 = 1;

    let pwr_ctl = (SD_DRIVER_ADDR + 0x29) as *mut u8;

    unsafe {
        *pwr_ctl = match level {
            PowerLevel::V33 => SD_BUS_VSEL_3V3_MASK | SD_BUS_PWR_MASK,
            PowerLevel::V30 => SD_BUS_VSEL_3V0_MASK | SD_BUS_PWR_MASK,
            PowerLevel::V18 => SD_BUS_VSEL_1V8_MASK | SD_BUS_PWR_MASK,
            PowerLevel::Close => 0,
        };
        if level == PowerLevel::V18 {
            *(0x030001F4 as *mut u8) = 0xd;
            mmio_setbits_32(REG_SDIO0_CLK_PAD_REG as _, (1 << 5) | (1 << 6) | (1 << 7));
        } else {
            *(0x030001F4 as *mut u8) = 0x9;
        }
        for _ in 0..0x100_0000 {
            asm!("nop")
        }
    }
}

pub fn set_clock(dividor: u8) {
    // try to set clock.
    unsafe {
        let clk_ctl = reg_transfer::<ClkCtl>(0x2c);
        clk_ctl.sd_clk_en().set(u1!(0));
        // set clock freq, out = internal_clock_freq / (2 x freq_sel)
        clk_ctl.freq_sel().set(dividor);
        clk_ctl.int_clk_en().set(u1!(1));
        loop {
            if clk_ctl.int_clk_stable().get() == u1!(1) {
                break;
            }
            for _ in 0..0x100_0000 {
                asm!("nop")
            }
        }
        clk_ctl.sd_clk_en().set(u1!(1));
        for _ in 0..0x100_0000 {
            asm!("nop")
        }
    }
}

pub fn close_clock() {
    // try to shutdown sdio clock.
    unsafe {
        let present_state = reg_transfer::<PresentState>(0x24);
        println!("present_state: {:#x?}", present_state);
        if present_state.cmd_inhibit().get() == false
            && present_state.dat_line_active().get() == u1!(0)
        {
            println!("CLK_CTL[SD_CLK_EN]=0 Close sdio clock");
            reg_transfer::<ClkCtl>(0x2c).sd_clk_en().set(u1!(0));
        }
        for _ in 0..0x100_0000 {
            asm!("nop")
        }
    }
}

pub fn read_buff(data: &mut [u8]) -> Result<(), CmdError> {
    assert!(data.len() == 0x200);
    let norm_int_sts = reg_transfer::<NormAndErrIntSts>(0x30);

    loop {
        if norm_int_sts.buf_rrdy().get() == true {
            mmio_write_32((SD_DRIVER_ADDR + 0x30) as _, 1 << 5);
            break;
        }
        if norm_int_sts.err_int().get() == true {
            mmio_write_32((SD_DRIVER_ADDR + 0x30) as _, 1 << 15);
            return Err(CmdError::IntError);
        }
        for _ in 0..1000 {
            unsafe { asm!("nop") }
        }
    }

    unsafe {
        core::slice::from_raw_parts_mut(data.as_mut_ptr() as *mut u32, 0x200 / 4)
            .iter_mut()
            .for_each(|x| {
                *x = mmio_read_32((SD_DRIVER_ADDR + 0x20) as _);
                asm!("nop");
            });
    }
    Ok(())
}

pub fn init() {
    // Initialize sd card gpio
    if check_sd() {
        pad_settings();
        reset_config();
        get_sd_caps();
        power_config(PowerLevel::V18);
        set_clock(4);

        // sdcard initialize.
        cmd_transfer(CommandType::CMD(0), 0, 0);
        cmd_transfer(CommandType::CMD(8), 0x1aa, 0);
        let args = 0x4000_0000 | 0x0030_0000 | (0x1FF << 15);
        // wait for initialization to end.
        loop {
            cmd_transfer(CommandType::CMD(55), 0, 0);
            cmd_transfer(CommandType::ACMD(41), args, 0);

            if *reg_transfer::<u32>(0x10) >> 31 == 1 {
                break;
            }
            for _ in 0..0x100_0000 { unsafe { asm!("nop") } }
        }
        // // get card and select
        cmd_transfer(CommandType::CMD(2), 0, 0);
        cmd_transfer(CommandType::CMD(3), 0, 0);
        let rsa = *reg_transfer::<u32>(0x10) & 0xffff0000;
        cmd_transfer(CommandType::CMD(9), rsa, 0); // get scd reg
        cmd_transfer(CommandType::CMD(7), rsa, 0); // select card

        
        // support 5 bit bus width.
        cmd_transfer(CommandType::CMD(55), rsa, 0);
        cmd_transfer(CommandType::ACMD(6), 2, 0).expect("can't send bus width");
        unsafe {
            *((SD_DRIVER_ADDR + 0x28) as *mut u8) |= 2;
        }
        // cmd_transfer(17, 0);
        // cmd_transfer(55, 0);
        // cmd_transfer(51, 0);
        // read();
        // dma_read();
        // cmd_transfer(CommandType::CMD(55), 0);
        // cmd_transfer(CommandType::ACMD(51), 0b10);

        read_scd(rsa).expect("can't do acmd 51");

        // tests();
    }
    panic!("manual shutdown @ cv1811-sd");
}

pub fn tests() {
    // test for read
    let mut data = vec![0u8; 512];

    for i in 0..2 {
        read_block(i, &mut data).expect("can't read block in tests");
        hexdump(&data);
    }
    log::debug!("读取成功");
}
