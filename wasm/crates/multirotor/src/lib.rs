//! Multirotor kernels for `wasm32-unknown-unknown` with `simd128`: the rotor
//! plant (`src/rotorPlant.ts`) and the flight controller
//! (`src/flightController.ts`). Stepping never allocates.
//!
//! Raw C ABI, version 1. The host allocates f64 buffers once with `mr_alloc`
//! and passes `(pointer, length)` pairs; every pair must name a whole live
//! allocation, and a buffer a kernel writes must not be passed twice. Each
//! kernel checks everything before it writes, returns 0 or an error code, and
//! on error leaves every buffer as it was.

// `!(x > 0.0)` is deliberate where NaN must fail, as in the TypeScript.
#![allow(clippy::neg_cmp_op_on_partial_ord)]

pub mod control;
pub mod plant;

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::RefCell;
use std::sync::atomic::{AtomicU32, Ordering};

/// A pointer or length does not name a live allocation, a written buffer is
/// aliased, or the lengths do not fit together.
pub const INVALID: u32 = 1;
pub const ALLOCATION: u32 = 2;
/// A value is non-finite, out of range or an unknown mode.
pub const INPUT: u32 = 3;

const MAX_BYTES: usize = 16 * 1024 * 1024;
const MAX_BUFFERS: usize = 64;

/// Counts every heap allocation, so a host can prove a step made none.
struct Counting;
static ALLOCATIONS: AtomicU32 = AtomicU32::new(0);
unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        ALLOCATIONS.fetch_add(1, Ordering::Relaxed);
        unsafe { System.alloc(layout) }
    }
    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        ALLOCATIONS.fetch_add(1, Ordering::Relaxed);
        unsafe { System.alloc_zeroed(layout) }
    }
    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, size: usize) -> *mut u8 {
        ALLOCATIONS.fetch_add(1, Ordering::Relaxed);
        unsafe { System.realloc(ptr, layout, size) }
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { System.dealloc(ptr, layout) }
    }
}
#[global_allocator]
static GLOBAL: Counting = Counting;

struct Registry {
    buffers: Vec<Vec<f64>>,
    bytes: usize,
    error: u32,
}

type View = (*mut f64, usize);

impl Registry {
    const fn new() -> Self {
        Self { buffers: Vec::new(), bytes: 0, error: 0 }
    }
    fn alloc(&mut self, length: u32) -> Result<usize, u32> {
        let len = length as usize;
        let bytes = len.checked_mul(8).ok_or(ALLOCATION)?;
        if len == 0 || self.buffers.len() >= MAX_BUFFERS || bytes > MAX_BYTES - self.bytes {
            return Err(ALLOCATION);
        }
        // The registry's own storage is reserved once, never while stepping.
        if self.buffers.capacity() == 0 {
            self.buffers.try_reserve_exact(MAX_BUFFERS).map_err(|_| ALLOCATION)?;
        }
        let mut buffer = Vec::new();
        buffer.try_reserve_exact(len).map_err(|_| ALLOCATION)?;
        buffer.resize(len, 0.0);
        let pointer = buffer.as_ptr() as usize;
        self.buffers.push(buffer);
        self.bytes += bytes;
        Ok(pointer)
    }
    fn free(&mut self, pointer: usize, length: u32) -> Result<(), u32> {
        let index = self
            .buffers
            .iter()
            .position(|b| b.as_ptr() as usize == pointer && b.len() == length as usize)
            .ok_or(INVALID)?;
        let buffer = self.buffers.swap_remove(index);
        self.bytes -= buffer.len() * 8;
        Ok(())
    }
    /// Resolves `(pointer, length, written)` arguments to whole allocations.
    fn views<const N: usize>(&mut self, args: [(usize, u32, bool); N]) -> Result<[View; N], u32> {
        let mut views = [(core::ptr::null_mut(), 0); N];
        for (view, &(pointer, length, _)) in views.iter_mut().zip(&args) {
            let buffer = self
                .buffers
                .iter_mut()
                .find(|b| b.as_ptr() as usize == pointer && b.len() == length as usize)
                .ok_or(INVALID)?;
            *view = (buffer.as_mut_ptr(), buffer.len());
        }
        for (i, a) in args.iter().enumerate() {
            if a.2 && args.iter().enumerate().any(|(j, b)| i != j && a.0 == b.0) {
                return Err(INVALID);
            }
        }
        Ok(views)
    }
}

/// # Safety
/// The view comes from [`Registry::views`] in the same registry borrow, and
/// no written view aliases it.
unsafe fn read<'a>(view: View) -> &'a [f64] {
    unsafe { core::slice::from_raw_parts(view.0, view.1) }
}
/// # Safety
/// As [`read`], and the view aliases no other view.
unsafe fn write<'a>(view: View) -> &'a mut [f64] {
    unsafe { core::slice::from_raw_parts_mut(view.0, view.1) }
}

thread_local! { static REGISTRY: RefCell<Registry> = const { RefCell::new(Registry::new()) }; }

fn with(run: impl FnOnce(&mut Registry) -> Result<(), u32>) -> u32 {
    REGISTRY.with(|r| match r.try_borrow_mut() {
        Ok(mut r) => {
            let status = run(&mut r).err().unwrap_or(0);
            r.error = status;
            status
        }
        Err(_) => INVALID,
    })
}

fn check(condition: bool, code: u32) -> Result<(), u32> {
    if condition { Ok(()) } else { Err(code) }
}

#[unsafe(no_mangle)]
pub extern "C" fn mr_abi_version() -> u32 {
    1
}

#[unsafe(no_mangle)]
pub extern "C" fn mr_last_error() -> u32 {
    REGISTRY.with(|r| r.try_borrow().map_or(INVALID, |r| r.error))
}

/// Heap allocations since instantiation.
#[unsafe(no_mangle)]
pub extern "C" fn mr_heap_allocations() -> u32 {
    ALLOCATIONS.load(Ordering::Relaxed)
}

/// A zeroed f64 buffer of `length`; 0 on failure (see `mr_last_error`).
#[unsafe(no_mangle)]
pub extern "C" fn mr_alloc(length: u32) -> usize {
    let mut pointer = 0;
    with(|r| {
        pointer = r.alloc(length)?;
        Ok(())
    });
    pointer
}

#[unsafe(no_mangle)]
pub extern "C" fn mr_free(pointer: usize, length: u32) -> u32 {
    with(|r| r.free(pointer, length))
}

/// `stepRotorPlant(config, input, state, output)`, 1 to 8 rotors (see [`plant`]).
#[unsafe(no_mangle)]
#[allow(clippy::too_many_arguments)]
pub extern "C" fn mr_plant(
    config: usize,
    config_len: u32,
    input: usize,
    input_len: u32,
    state: usize,
    state_len: u32,
    output: usize,
    output_len: u32,
) -> u32 {
    with(|r| {
        let n = plant::rotors(config_len as usize, input_len as usize, state_len as usize, output_len as usize).ok_or(INVALID)?;
        let [c, i, s, o] = r.views([(config, config_len, false), (input, input_len, false), (state, state_len, true), (output, output_len, true)])?;
        // SAFETY: views from this borrow; written views alias nothing.
        let (config, input, state, output) = unsafe { (read(c), read(i), write(s), write(o)) };
        check(plant::valid(config, input, state), INPUT)?;
        plant::step(config, input, state, output, n);
        Ok(())
    })
}

/// `stepFlightController(config, input, state, output)`, 1 to 8 rotors (see [`control`]).
#[unsafe(no_mangle)]
#[allow(clippy::too_many_arguments)]
pub extern "C" fn mr_control(
    config: usize,
    config_len: u32,
    input: usize,
    input_len: u32,
    state: usize,
    state_len: u32,
    output: usize,
    output_len: u32,
) -> u32 {
    with(|r| {
        let n = control::rotors(config_len as usize, input_len as usize, state_len as usize, output_len as usize).ok_or(INVALID)?;
        let [c, i, s, o] = r.views([(config, config_len, false), (input, input_len, false), (state, state_len, true), (output, output_len, true)])?;
        // SAFETY: views from this borrow; written views alias nothing.
        let (config, input, state, output) = unsafe { (read(c), read(i), write(s), write(o)) };
        check(control::valid(config, input, state), INPUT)?;
        control::step(config, input, state, output, n);
        Ok(())
    })
}
