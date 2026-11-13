// The commissioning record -- the numbers a commissioning run measures, turned into bytes, stored
// across a power cycle, and read back by a host that was not there when they were measured. Every
// call in this file is a `foc_*` function: a C host writes the same lines.
//
// Build it the way the `demo` gate does -- the example calls into the library, so the sources are
// part of the command, and `-Iinclude` alone fails at the link. A renamed source turns that gate red
// instead of leaving a stale instruction here:
//   g++ -std=c++17 -fno-exceptions -fno-rtti -Wall -Wextra -Werror -Iinclude
//   docs/doxygen/dox/demos/commissioning-record.cxx src/core/*.cpp
//   src/core/observer/*.cpp src/motor/*.cpp src/driver/*.cpp src/api/foc_api.c
//   src/api/bridge.cpp src/version.c
//   -o /tmp/commissioning-record
//
// 配置记录 —— 把一次标定量到的数字变成字节、跨过一次断电存下来，再由一台当时并不在场的宿主读回去。
// 本文件里的每次调用都是 `foc_*` 函数：一个 C 宿主写的是同样的几行。

#include <cmath>
#include <cstdint>
#include <cstdio>

#include <foc/api/foc_api.h>

namespace {

// What a commissioning run on this machine actually measured. In a real line these come out of the
// estimators; here they are typed in so the example is about the record and not about the fit.
auto measured_set() -> foc_commissioned_set_t {
  foc_commissioned_set_t set{};
  set.struct_size = sizeof(set);
  set.machine_id = 0x5150u; // the reader refuses a set that belongs to another machine
  set.params.rs = 0.42F;
  set.params.ld = 0.00031F;
  set.params.lq = 0.00042F;
  set.params.kt = 0.061F;
  set.params.j = 1.1e-4F;
  set.params.b = 2.0e-6F;
  set.params.pole_pairs = 7;
  // Both of these are optional in the record on purpose: "not stated" is information, and a
  // plausible default is not.
  set.measured_at_unix_s = 0;
  set.temperature_c = NAN;
  return set;
}

} // namespace

auto main() -> int {
  const foc_commissioned_set_t measured = measured_set();

  // ---- write -------------------------------------------------------------------------------
  uint8_t record[64]{};
  uint32_t written = 0;
  const foc_err_t encoded = foc_commissioned_set_encode(&measured, record, sizeof(record), &written);
  if (encoded != FOC_OK) {
    std::printf("encode refused: %d\n", static_cast<int>(encoded));
    return 1;
  }
  std::printf("encoded %u bytes for machine 0x%04X\n", static_cast<unsigned>(written),
              static_cast<unsigned>(measured.machine_id));

  // A buffer that is too small is a *different* refusal from numbers that cannot be true, which is
  // why the two answer different codes: one means "give me more room", the other "measure again".
  uint8_t small[8]{};
  const foc_err_t too_small = foc_commissioned_set_encode(&measured, small, sizeof(small), nullptr);
  std::printf("a 8-byte buffer answers %s\n",
              too_small == FOC_ERR_NO_RESOURCE ? "NO_RESOURCE (give me more room)" : "something else");

  // ---- the power cycle ---------------------------------------------------------------------
  // Nothing in the struct survives: only the bytes do. This is the whole point of the record, and
  // the reason it exists in the C face rather than behind a C++ helper.
  foc_commissioned_set_t reloaded{};

  // ---- read back ---------------------------------------------------------------------------
  foc_set_status_t status = FOC_SET_OK;
  uint32_t bad_offset = 0;
  const foc_err_t decoded =
      foc_commissioned_set_decode(record, written, &reloaded, &status, &bad_offset);
  if (decoded != FOC_OK) {
    std::printf("decode refused at byte %u: status %d\n", static_cast<unsigned>(bad_offset),
                static_cast<int>(status));
    return 1;
  }
  std::printf("read back rs=%.4f ld=%.6f lq=%.6f kt=%.4f pole_pairs=%u\n", static_cast<double>(reloaded.params.rs),
              static_cast<double>(reloaded.params.ld), static_cast<double>(reloaded.params.lq),
              static_cast<double>(reloaded.params.kt), static_cast<unsigned>(reloaded.params.pole_pairs));
  std::printf("unstated fields stay unstated: measured_at=%u temperature=%s\n",
              static_cast<unsigned>(reloaded.measured_at_unix_s),
              std::isnan(reloaded.temperature_c) ? "NaN" : "a number");

  // ---- one corrupted byte ------------------------------------------------------------------
  // A single flipped bit in the payload is the case a check value exists for, and the answer names
  // the byte rather than describing the damage.
  uint8_t corrupted[64]{};
  for (uint32_t i = 0; i < written; ++i) {
    corrupted[i] = record[i];
  }
  corrupted[written / 2] ^= 0x01u;
  foc_commissioned_set_t untouched{};
  untouched.machine_id = 0xDEADu; // must survive the refusal: "every refusal leaves the receiver alone"
  foc_set_status_t bad_status = FOC_SET_OK;
  uint32_t bad_byte = 0;
  const foc_err_t refused =
      foc_commissioned_set_decode(corrupted, written, &untouched, &bad_status, &bad_byte);
  std::printf("a flipped bit answers %s at byte %u (status %d), receiver still 0x%04X\n",
              refused == FOC_OK ? "OK" : "PARAM", static_cast<unsigned>(bad_byte),
              static_cast<int>(bad_status), static_cast<unsigned>(untouched.machine_id));
  return 0;
}
