/*
 * stackpaint.h - core1 stack high-water mark (development aid)
 *
 * Assumes: bool core1_separate_stack = true; (core1 has its own 8KB stack)
 *
 * How it works:
 *   - stackpaint_core1() runs on core1, first thing in setup1(). It reads the
 *     current stack pointer and fills a region below it with a sentinel word.
 *     The stack grows downward, so everything the DSP ever uses will overwrite
 *     paint from the top down.
 *   - stackpaint_check() runs on core0 (UI). It scans upward from the bottom
 *     of the painted region and counts how many sentinel words are still
 *     intact. That count is the margin (in bytes) core1 has never needed.
 *
 * Notes:
 *   - PAINT_SIZE is 7KB of the 8KB block. setup1() is entered within a couple
 *     of hundred bytes of the top of the stack, so 7KB + guard stays safely
 *     inside the allocation and never scribbles on the heap below it. The
 *     unpainted ~800 bytes at the bottom mean the reported margin slightly
 *     UNDER-estimates the true margin - conservative in the right direction.
 *   - If ever used without core1_separate_stack (shared 4K stacks), reduce
 *     STACKPAINT_SIZE to 3KB or the painter will walk into core-adjacent RAM.
 *   - The watermark is monotonic: it only ever gets worse. Exercise every deep
 *     path (menu, both CW decoders, NB max, FT8 decode, SSB TX with proc and
 *     CESSB, a settings save) before trusting the number.
 *   - An IRQ landing mid-paint just stamps its frame near the top of the
 *     region, which is harmless - that area counts as "used" immediately
 *     anyway.
 */

#ifndef STACKPAINT_H
#define STACKPAINT_H

#include <Arduino.h>

#define STACKPAINT_SENTINEL 0xC0DE57ACul
#define STACKPAINT_SIZE     (3u * 1024u) // painted span, must fit the stack
#define STACKPAINT_GUARD    128u         // untouched zone just below live SP

volatile static uint32_t *stackpaint_lo = nullptr; // lowest painted word
volatile static uint32_t *stackpaint_hi = nullptr; // top of paint (exclusive)

// Call as the FIRST statement in setup1(), before anything else runs deep.
// noinline so the asm reads the SP of a real, settled frame.
static void __attribute__((noinline)) stackpaint_core(void)
{
  uint32_t sp = 0;
  asm volatile ("mov %0, sp" : "=r" (sp));
  const uint32_t hi = (sp - STACKPAINT_GUARD) & ~3ul; // word aligned
  const uint32_t lo = hi - STACKPAINT_SIZE;
  volatile uint32_t *p = (volatile uint32_t *)lo;
  while ((uint32_t)p < hi)
  {
    *p++ = STACKPAINT_SENTINEL;
  }
  // publish only after painting is complete
  stackpaint_lo = (volatile uint32_t *)lo;
  stackpaint_hi = (volatile uint32_t *)hi;
}

// Call from loop() on core0, e.g. once per second. Returns bytes of paint
// still intact at the bottom of core1's stack (0 = paint fully consumed,
// overflow imminent or already happened - grow the stack immediately).
static uint32_t stackpaint_check(void)
{
  if (stackpaint_lo == nullptr)
  {
    // core1 hasn't painted yet
    return 0;
  }
  volatile uint32_t *p = stackpaint_lo;
  uint32_t untouched = 0;
  while (p < stackpaint_hi && *p == STACKPAINT_SENTINEL)
  {
    untouched += 4u;
    p++;
  }
  return untouched;
}

// Convenience: deepest stack usage seen so far, in bytes, measured from the
// top of the painted span. (Excludes the guard and the unpainted slack, so
// it slightly over-estimates - again conservative.)
static uint32_t stackpaint_used(void)
{
  return STACKPAINT_SIZE - stackpaint_check();
}

#endif // STACKPAINT_H
