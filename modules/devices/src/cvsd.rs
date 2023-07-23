use alloc::sync::Arc;
use fdt::node::FdtNode;

use crate::{
    device::{BlkDriver, Driver},
    BLK_DEVICES, DRIVER_REGS,
};

pub struct CvSd;

unsafe impl Sync for CvSd {}
unsafe impl Send for CvSd {}

impl Driver for CvSd {
    fn device_type(&self) -> crate::device::DeviceType {
        crate::device::DeviceType::Block
    }

    fn get_id(&self) -> &str {
        "cvitek,sd"
    }

    fn as_blk(self: Arc<Self>) -> Option<Arc<dyn BlkDriver>> {
        Some(self.clone())
    }
}

impl BlkDriver for CvSd {
    fn read_block(&self, block_id: usize, buf: &mut [u8]) {
        cv1811_sd::read_block(block_id as _, buf).expect("can't read block by using CvSd");
    }

    fn write_block(&self, block_id: usize, buf: &[u8]) {
        // unimplemented!("cv sd write");
        cv1811_sd::write_block(block_id as _, buf).expect("can't write block by using CvSd");
    }
}

pub fn init_rtc(_node: &FdtNode) {
    let blk = CvSd;
    cv1811_sd::init().expect("init with err");
    BLK_DEVICES.lock().push(Arc::new(blk));
    info!("Initailize virtio-block device");
}

pub fn driver_init() {
    DRIVER_REGS.lock().insert("cvitek,mars-sd", init_rtc);
}