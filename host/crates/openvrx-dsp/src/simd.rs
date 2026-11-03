// SPDX-License-Identifier: GPL-3.0-or-later
//! Runtime CPU-feature dispatch for the hot loops.
//!
//! Kernels are written as plain loops over slices (LLVM vectorises them).
//! [`dispatch!`] compiles each kernel twice, once for the baseline target
//! and once with AVX2 + FMA enabled, and picks the AVX2 build at run time
//! when the CPU has it. No unsafe code is needed by callers.

/// True when the CPU supports AVX2 and FMA (cached by std).
#[inline]
pub fn has_avx2() -> bool {
    #[cfg(target_arch = "x86_64")]
    {
        std::is_x86_feature_detected!("avx2") && std::is_x86_feature_detected!("fma")
    }
    #[cfg(not(target_arch = "x86_64"))]
    {
        false
    }
}

/// Defines a function whose body is compiled for the baseline target and
/// for AVX2+FMA, dispatched at run time.
macro_rules! dispatch {
    ($(#[$m:meta])* $vis:vis fn $name:ident($($a:ident : $t:ty),* $(,)?) $(-> $r:ty)? $body:block) => {
        $(#[$m])*
        $vis fn $name($($a: $t),*) $(-> $r)? {
            #[inline(always)]
            fn inner($($a: $t),*) $(-> $r)? $body
            #[cfg(target_arch = "x86_64")]
            {
                #[target_feature(enable = "avx2,fma")]
                unsafe fn avx($($a: $t),*) $(-> $r)? {
                    inner($($a),*)
                }
                if $crate::simd::has_avx2() {
                    // SAFETY: the CPU supports AVX2 and FMA (checked above).
                    return unsafe { avx($($a),*) };
                }
            }
            inner($($a),*)
        }
    };
}
