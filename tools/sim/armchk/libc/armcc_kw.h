/* emulate armcc keywords for clang syntax checking */
#define __align(x) __attribute__((aligned(x)))
#define __forceinline __attribute__((always_inline)) inline
#define __weak __attribute__((weak))
