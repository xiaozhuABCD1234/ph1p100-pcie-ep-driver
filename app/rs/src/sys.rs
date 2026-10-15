// SPDX-License-Identifier: GPL-2.0
//! 原 C 版用到的那几个系统调用/库函数的 Rust 侧声明。
//!
//! 只声明必须声明的（ioctl / mmap / munmap / flock / strerror）；文件与目录
//! （sysfs 读取、节点打开、PGM 输出）一律用 std，不再自己声明 open/read/write。
//! 这里没有引入任何 crate，也没有用 bindgen。

use std::ffi::{c_char, c_int, c_void};
use std::io;

// Linux 常量：值取自 asm-generic（x86_64 与 aarch64 同值）。
pub const PROT_READ: c_int = 0x1;
pub const PROT_WRITE: c_int = 0x2;
pub const MAP_SHARED: c_int = 0x01;
pub const LOCK_EX: c_int = 0x2;
pub const LOCK_NB: c_int = 0x4;
/// `O_SYNC`（值取自 asm-generic/fcntl.h：0o4010000）
pub const O_SYNC: i32 = 0o4010000;

/// mmap 失败时 glibc 返回的 MAP_FAILED = (void *)-1
const MAP_FAILED: isize = -1;

extern "C" {
    fn ioctl(fd: c_int, request: u64, ...) -> c_int;
    fn mmap(
        addr: *mut c_void,
        length: usize,
        prot: c_int,
        flags: c_int,
        fd: c_int,
        offset: i64,
    ) -> *mut c_void;
    fn munmap(addr: *mut c_void, length: usize) -> c_int;
    fn flock(fd: c_int, operation: c_int) -> c_int;
    fn strerror(errnum: c_int) -> *mut c_char;
}

/// `strerror(errno)`：C 版每条出错信息里的 `%s` 都是它，这里保持一致。
pub fn errno_str(e: &io::Error) -> String {
    let n = e.raw_os_error().unwrap_or(0);
    // SAFETY: strerror 返回指向静态字符串的指针（本程序单线程），取不到就退回 errno 数字。
    unsafe {
        let p = strerror(n);
        if p.is_null() {
            return format!("errno {}", n);
        }
        std::ffi::CStr::from_ptr(p).to_string_lossy().into_owned()
    }
}

/// `ioctl(fd, request, arg)`：失败时把 errno 包成 io::Error。
///
/// # Safety
/// `arg` 必须指向与 `request` 对应的、驱动会写入的那个结构体（按 ioctl 协议的 ABI 布局）。
pub unsafe fn ioctl_ptr(fd: c_int, request: u64, arg: *mut c_void) -> io::Result<()> {
    // SAFETY: 由调用方保证（见上面的 Safety 一节）
    let r = unsafe { ioctl(fd, request, arg) };
    if r < 0 {
        Err(io::Error::last_os_error())
    } else {
        Ok(())
    }
}

/// `flock(fd, LOCK_EX | LOCK_NB)`（一次只允许一个进程持有设备）
pub fn flock_exclusive_nb(fd: c_int) -> io::Result<()> {
    // SAFETY: fd 是本进程已打开的有效描述符。
    let r = unsafe { flock(fd, LOCK_EX | LOCK_NB) };
    if r != 0 {
        Err(io::Error::last_os_error())
    } else {
        Ok(())
    }
}

/// 一块 mmap 出来的内存；离开作用域即 munmap（等价于 C 版各条路径上的 munmap）。
pub struct Mmap {
    ptr: *mut u8,
    len: usize,
}

impl Mmap {
    /// `mmap(NULL, len, prot, MAP_SHARED, fd, 0)`
    pub fn shared(fd: c_int, len: usize, prot: c_int) -> io::Result<Mmap> {
        // SAFETY: 参数与 C 版一致（offset=0、MAP_SHARED）。
        let p = unsafe { mmap(std::ptr::null_mut(), len, prot, MAP_SHARED, fd, 0) };
        if p as isize == MAP_FAILED || p.is_null() {
            return Err(io::Error::last_os_error());
        }
        Ok(Mmap { ptr: p as *mut u8, len })
    }

    pub fn as_ptr(&self) -> *mut u8 {
        self.ptr
    }
}

impl Drop for Mmap {
    fn drop(&mut self) {
        // SAFETY: ptr/len 就是本对象 mmap 出来的那一对。
        unsafe {
            munmap(self.ptr as *mut c_void, self.len);
        }
    }
}

/// `volatile uint32_t` 读（BAR 窗口 / mailbox 槽头）
///
/// # Safety
/// `p` 必须是 4 字节对齐且可读的有效地址。
pub unsafe fn rd32(p: *const u32) -> u32 {
    p.read_volatile()
}

/// `volatile uint32_t` 写，后面跟一道全屏障（对应 C 版的 `__sync_synchronize()`）
///
/// # Safety
/// `p` 必须是 4 字节对齐且可写的有效地址。
pub unsafe fn wr32(p: *mut u32, v: u32) {
    p.write_volatile(v);
    std::sync::atomic::fence(std::sync::atomic::Ordering::SeqCst);
}
