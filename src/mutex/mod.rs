//! Our mutex implementation.
//!
//! Unlock the cortex-m mutex, it panics on collision, rather than disabling
//! interrupts while the lock is held.

use atomic_polyfill::{AtomicBool, Ordering};

/// A simple no-std mutex.
///
/// Uses critical-section to hold an atomic bool, for when you don't have
/// atomic-compare-swap.
pub struct NeoMutex<T> {
	locked: atomic_polyfill::AtomicBool,
	value: core::cell::UnsafeCell<T>,
}

impl<T> NeoMutex<T> {
	/// Create a new Mutex.
	///
	/// Defaults to unlocked.
	pub const fn new(value: T) -> NeoMutex<T> {
		NeoMutex {
			locked: AtomicBool::new(false),
			value: core::cell::UnsafeCell::new(value),
		}
	}

	/// Lock the mutex.
	///
	/// Unlock it by dropping the returned object.
	///
	/// Panics if the mutex is already locked.
	pub fn lock<'a>(&'a self) -> NeoMutexGuard<'a, T> {
		let _ = self
			.locked
			.compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed)
			.unwrap();
		NeoMutexGuard { parent: self }
	}
}

unsafe impl<T> Sync for NeoMutex<T> {}

/// Represents a locked mutex.
///
/// Is unlocked on drop.
pub struct NeoMutexGuard<'a, T> {
	parent: &'a NeoMutex<T>,
}

impl<'a, T> Drop for NeoMutexGuard<'a, T> {
	fn drop(&mut self) {
		self.parent.locked.store(false, Ordering::Release);
	}
}

impl<'a, T> core::ops::Deref for NeoMutexGuard<'a, T> {
	type Target = T;

	fn deref(&self) -> &Self::Target {
		unsafe { &*self.parent.value.get() }
	}
}

impl<'a, T> core::ops::DerefMut for NeoMutexGuard<'a, T> {
	fn deref_mut(&mut self) -> &mut Self::Target {
		unsafe { &mut *self.parent.value.get() }
	}
}
