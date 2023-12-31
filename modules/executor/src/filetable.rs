use core::ops::{Deref, DerefMut};

use alloc::{
    string::{String, ToString},
    sync::Arc,
    vec::Vec,
};
use fs::{
    dentry::{self, dentry_open, dentry_root, DentryNode},
    INodeInterface, VfsError, WaitBlockingRead, WaitBlockingWrite,
};
use sync::Mutex;
use vfscore::{
    DirEntry, MMapFlags, Metadata, OpenFlags, PollEvent, SeekFrom, Stat, StatFS, TimeSpec,
};

const FILE_MAX: usize = 255;
const FD_NONE: Option<Arc<FileItem>> = Option::None;

#[derive(Clone)]
pub struct FileTable(pub Vec<Option<Arc<FileItem>>>);

impl FileTable {
    pub fn new() -> Self {
        let mut file_table: Vec<Option<Arc<FileItem>>> = vec![FD_NONE; FILE_MAX];
        file_table[..3].fill(Some(
            FileItem::fs_open("/dev/ttyv0", OpenFlags::O_RDWR).expect("can't read tty file"),
        ));
        Self(file_table)
    }
}

impl Deref for FileTable {
    type Target = Vec<Option<Arc<FileItem>>>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for FileTable {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

pub fn rlimits_new() -> Vec<usize> {
    let mut rlimits = vec![0usize; 8];
    rlimits[7] = FILE_MAX;
    rlimits
}

bitflags! {
    #[derive(Clone, Debug)]
    pub struct FileOptions: u8 {
        /// Hangup.
        const R = 1;
        const W = 1 << 1;
        const X = 1 << 3;
        /// Create file.
        const C = 1 << 4;
    }
}

impl Default for FileOptions {
    fn default() -> Self {
        FileOptions::R | FileOptions::W | FileOptions::X
    }
}

pub struct FileItem {
    pub path: String,
    pub inner: Arc<dyn INodeInterface>,
    pub dentry: Option<Arc<DentryNode>>,
    pub options: FileOptions,
    pub offset: Mutex<usize>,
    pub flags: Mutex<OpenFlags>,
}

impl<'a> FileItem {
    pub fn new(
        inner: Arc<dyn INodeInterface>,
        dentry: Option<Arc<DentryNode>>,
        options: FileOptions,
    ) -> Arc<Self> {
        Arc::new(Self {
            path: String::new(),
            inner,
            options,
            dentry,
            offset: Mutex::new(0),
            flags: Mutex::new(OpenFlags::NONE),
        })
    }

    pub fn new_dev(inner: Arc<dyn INodeInterface>) -> Arc<Self> {
        Arc::new(Self {
            path: String::new(),
            inner,
            offset: Mutex::new(0),
            dentry: None,
            options: FileOptions::default(),
            flags: Mutex::new(OpenFlags::NONE),
        })
    }

    pub fn get_bare_file(&self) -> Arc<dyn INodeInterface> {
        self.inner.clone()
    }

    pub fn fs_open(path: &str, open_flags: OpenFlags) -> Result<Arc<Self>, VfsError> {
        let mut options = FileOptions::R | FileOptions::X;
        if open_flags.contains(OpenFlags::O_WRONLY)
            || open_flags.contains(OpenFlags::O_RDWR)
            || open_flags.contains(OpenFlags::O_ACCMODE)
        {
            options = options.union(FileOptions::W);
        }
        let dentry_node = dentry::dentry_open(dentry_root(), path, OpenFlags::NONE)?;
        let offset = if open_flags.contains(OpenFlags::O_APPEND) {
            dentry_node.node.metadata()?.size
        } else {
            0
        };
        Ok(Arc::new(Self {
            path: path.to_string(),
            inner: dentry_node.node.clone(),
            options,
            dentry: Some(dentry_node),
            offset: Mutex::new(offset),
            flags: Mutex::new(open_flags),
        }))
    }

    pub fn dentry_open(&self, path: &str, flags: OpenFlags) -> Result<Arc<Self>, VfsError> {
        assert!(self.dentry.is_some());
        dentry_open(self.dentry.clone().unwrap(), path, flags)
            .map(|x| FileItem::new(x.node.clone(), Some(x), FileOptions::all()))
    }

    #[inline(always)]
    fn check_writeable(&self) -> Result<(), VfsError> {
        if self.options.contains(FileOptions::W) {
            Ok(())
        } else {
            Err(VfsError::NotWriteable)
        }
    }

    pub fn path(&'a self) -> Result<&'a str, VfsError> {
        Ok(&self.path)
    }
}

impl INodeInterface for FileItem {
    fn readat(&self, offset: usize, buffer: &mut [u8]) -> Result<usize, VfsError> {
        self.inner.readat(offset, buffer)
    }

    fn writeat(&self, offset: usize, buffer: &[u8]) -> Result<usize, VfsError> {
        self.check_writeable()?;
        if buffer.len() == 0 {
            return Ok(0);
        }
        self.inner.writeat(offset, buffer)
    }

    fn mkdir(&self, name: &str) -> Result<Arc<dyn INodeInterface>, VfsError> {
        self.inner.mkdir(name)
    }

    fn rmdir(&self, name: &str) -> Result<(), VfsError> {
        self.inner.rmdir(name)
    }

    fn remove(&self, name: &str) -> Result<(), VfsError> {
        self.inner.remove(name)
    }

    fn touch(&self, name: &str) -> Result<Arc<dyn INodeInterface>, VfsError> {
        self.inner.touch(name)
    }

    fn read_dir(&self) -> Result<Vec<DirEntry>, VfsError> {
        self.inner.read_dir()
    }

    fn metadata(&self) -> Result<Metadata, VfsError> {
        self.inner.metadata()
    }

    fn lookup(&self, name: &str) -> Result<Arc<dyn INodeInterface>, VfsError> {
        self.inner.lookup(name)
    }

    fn open(&self, name: &str, flags: OpenFlags) -> Result<Arc<dyn INodeInterface>, VfsError> {
        self.inner.open(name, flags)
    }

    fn ioctl(&self, command: usize, arg: usize) -> Result<usize, VfsError> {
        self.inner.ioctl(command, arg)
    }

    fn truncate(&self, size: usize) -> Result<(), VfsError> {
        // self.check_writeable()?;
        // let mut offset = self.offset.lock();
        // if *offset > size {
        //     *offset = size;
        // }
        self.inner.truncate(size)
    }

    fn flush(&self) -> Result<(), VfsError> {
        self.inner.flush()
    }

    fn resolve_link(&self) -> Result<String, VfsError> {
        self.inner.resolve_link()
    }

    fn link(&self, name: &str, src: Arc<dyn INodeInterface>) -> Result<(), VfsError> {
        self.inner.link(name, src)
    }

    fn unlink(&self, name: &str) -> Result<(), VfsError> {
        self.inner.unlink(name)
    }

    fn mmap(&self, offset: usize, size: usize, flags: MMapFlags) -> Result<usize, VfsError> {
        self.inner.mmap(offset, size, flags)
    }

    fn stat(&self, stat: &mut Stat) -> Result<(), VfsError> {
        self.inner.stat(stat)?;
        stat.dev = 0;
        Ok(())
    }

    fn mount(&self, path: &str) -> Result<(), VfsError> {
        self.inner.mount(path)
    }

    fn umount(&self) -> Result<(), VfsError> {
        self.inner.umount()
    }

    fn statfs(&self, statfs: &mut StatFS) -> Result<(), VfsError> {
        self.inner.statfs(statfs)
    }

    fn getdents(&self, buffer: &mut [u8]) -> Result<usize, VfsError> {
        self.inner.getdents(buffer)
    }

    fn utimes(&self, times: &mut [TimeSpec]) -> Result<(), VfsError> {
        self.inner.utimes(times)
    }

    fn poll(&self, events: PollEvent) -> Result<PollEvent, VfsError> {
        self.inner.poll(events)
    }
}

impl FileItem {
    pub fn read(&self, buffer: &mut [u8]) -> Result<usize, VfsError> {
        let offset = *self.offset.lock();
        self.inner.readat(offset, buffer).map(|x| {
            *self.offset.lock() += x;
            x
        })
    }

    pub fn write(&self, buffer: &[u8]) -> Result<usize, VfsError> {
        self.check_writeable()?;
        if buffer.len() == 0 {
            return Ok(0);
        }
        let offset = *self.offset.lock();
        self.inner.writeat(offset, buffer).map(|x| {
            *self.offset.lock() += x;
            x
        })
    }

    pub async fn async_read(&self, buffer: &mut [u8]) -> Result<usize, VfsError> {
        let offset = *self.offset.lock();
        if self.flags.lock().contains(OpenFlags::O_NONBLOCK) {
            self.inner.readat(offset, buffer)
        } else {
            WaitBlockingRead(self.inner.clone(), buffer, offset).await
        }
        .map(|x| {
            *self.offset.lock() += x;
            x
        })
    }

    pub async fn async_write(&self, buffer: &[u8]) -> Result<usize, VfsError> {
        // self.check_writeable()?;
        if buffer.len() == 0 {
            return Ok(0);
        }
        let offset = *self.offset.lock();
        WaitBlockingWrite(self.inner.clone(), &buffer, offset)
            .await
            .map(|x| {
                *self.offset.lock() += x;
                x
            })
    }

    pub fn seek(&self, seek_from: SeekFrom) -> Result<usize, VfsError> {
        let offset = *self.offset.lock();
        let mut new_off = match seek_from {
            SeekFrom::SET(off) => off as isize,
            SeekFrom::CURRENT(off) => offset as isize + off,
            SeekFrom::END(off) => self.metadata()?.size as isize + off,
        };
        if new_off < 0 {
            new_off = 0;
        }
        // assert!(new_off >= 0);
        *self.offset.lock() = new_off as _;
        Ok(new_off as _)
    }
}
