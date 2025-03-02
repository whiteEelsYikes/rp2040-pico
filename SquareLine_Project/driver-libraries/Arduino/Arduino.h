#include <stddef.h>
#include <stdint.h>
//#include <algorithm>
#include "Common.h"
#include "rpipicow/pins_arduino.h"
#include "debug_internal.h"


#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

// Disable/re-enable all interrupts.  Safely handles nested disables
void interrupts();
void noInterrupts();

#define sei() interrupts()
#define cli() noInterrupts()

#ifdef __cplusplus
} // extern "C"
#endif


#ifdef __cplusplus

// Template which will evaluate at *compile time* to a single 32b number
// with the specified bits set.
template <size_t N>
constexpr uint32_t __bitset(const int (&a)[N], size_t i = 0U) {
    return i < N ? (1L << a[i]) | __bitset(a, i + 1) : 0;
}

#endif
