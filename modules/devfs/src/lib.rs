#![no_std]

extern crate alloc;
extern crate log;

use alloc::{collections::BTreeMap, string::ToString, sync::Arc, vec::Vec};
use vfscore::{DirEntry, FileSystem, FileType, INodeInterface, MountedInfo, VfsError, VfsResult};

mod sdx;
mod stdin;
mod stdout;

pub use {sdx::Sdx, stdin::Stdin, stdout::Stdout};

pub struct DevFS {
    root_dir: Arc<dyn INodeInterface>,
}

impl DevFS {
    pub fn new() -> Arc<Self> {
        Arc::new(Self {
            root_dir: Arc::new(DevDir::new()),
        })
    }

    pub fn new_with_dir(dev: DevDir) -> Arc<Self> {
        Arc::new(Self {
            root_dir: Arc::new(dev),
        })
    }
}

impl FileSystem for DevFS {
    fn root_dir(&'static self, _mi: MountedInfo) -> Arc<dyn INodeInterface> {
        self.root_dir.clone()
    }

    fn name(&self) -> &str {
        "devfs"
    }
}

pub struct DevDir {
    map: BTreeMap<&'static str, Arc<dyn INodeInterface>>,
}

impl DevDir {
    pub fn new() -> Self {
        let mut map: BTreeMap<&'static str, Arc<dyn INodeInterface>> = BTreeMap::new();
        map.insert("stdout", Arc::new(stdout::Stdout));
        map.insert("stderr", Arc::new(stdout::Stdout));
        map.insert("stdin", Arc::new(stdin::Stdin));
        Self { map }
    }

    pub fn add(&mut self, path: &'static str, node: Arc<dyn INodeInterface>) {
        self.map.insert(path, node);
    }
}

impl INodeInterface for DevDir {
    fn open(&self, name: &str, _flags: vfscore::OpenFlags) -> VfsResult<Arc<dyn INodeInterface>> {
        self.map
            .get(name)
            .map(|x| x.clone())
            .ok_or(VfsError::FileNotFound)
    }

    fn read_dir(&self) -> VfsResult<Vec<DirEntry>> {
        Ok(self
            .map
            .iter()
            .map(|(name, _)| DirEntry {
                filename: name.to_string(),
                len: 0,
                file_type: FileType::Device,
            })
            .collect())
    }
}