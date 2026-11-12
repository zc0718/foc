// Telemetry -- reading the machine-readable registers out of a drive, putting them on a wire, and getting them back
// unchanged. Every call in this file is a `foc_*` function: a C host writes the same lines.
//
// Build it the way the `demo` gate does -- the example calls into the library, so the sources are part of the command,
// and `-Iinclude` alone fails at the link. A renamed source turns that gate red instead of leaving a stale instruction
// here:
//   g++ -std=c++17 -fno-exceptions -fno-rtti -Wall -Wextra -Werror -Iinclude
//   docs/doxygen/dox/demos/telemetry-loopback.cxx src/core/*.cpp
//   src/core/observer/*.cpp src/motor/*.cpp src/driver/*.cpp src/api/foc_api.c
//   src/api/bridge.cpp src/version.c
//   -o /tmp/telemetry-loopback
//
// 遥测 —— 把机器可读的寄存器从一个驱动器里读出来、放上线、再原样拿回来。本文件里的每次调用都是 `foc_*` 函数：
// 一个 C 宿主写的是同样的几行。

#include <cstdint>
#include <cstdio>

#include <foc/api/foc_api.h>

namespace {

constexpr float kDtS = 5.0e-5F;
constexpr int kTicks = 40; // two control milliseconds: enough for the state registers to have something to say

// Little-endian by construction rather than by trusting the host: this is the wire order, and a host that let the
// compiler choose would send a different frame from a big-endian board.
// 按构造用小端，而不是相信主机字节序：这是线上顺序，让编译器去选的宿主，从大端板子上发出的会是另一帧。
void put_u32(std::uint8_t* out, std::uint32_t word) {
    out[0] = static_cast<std::uint8_t>(word & 0xFFU);
    out[1] = static_cast<std::uint8_t>((word >> 8U) & 0xFFU);
    out[2] = static_cast<std::uint8_t>((word >> 16U) & 0xFFU);
    out[3] = static_cast<std::uint8_t>((word >> 24U) & 0xFFU);
}

std::uint32_t get_u32(const std::uint8_t* in) {
    return static_cast<std::uint32_t>(in[0]) | (static_cast<std::uint32_t>(in[1]) << 8U) |
           (static_cast<std::uint32_t>(in[2]) << 16U) | (static_cast<std::uint32_t>(in[3]) << 24U);
}

const char* frame_status_name(foc_frame_status_t status) {
    switch (status) {
    case FOC_FRAME_OK:
        return "ok";
    case FOC_FRAME_TOO_SHORT:
        return "too short";
    case FOC_FRAME_BAD_START:
        return "bad start byte";
    case FOC_FRAME_BAD_LENGTH:
        return "length past the buffer";
    case FOC_FRAME_BAD_CHECKSUM:
        return "checksum mismatch";
    }
    return "?";
}

} // namespace

int main() {
    const foc_motor_params_t params = {0.5F, 4e-4F, 4e-4F, 0.05F, 1e-4F, 1e-4F, 7U};
    foc_motor_t motor = nullptr;
    if (foc_motor_create(&params, &motor) != FOC_OK) {
        std::printf("could not create the drive\n");
        return 1;
    }
    (void)foc_motor_enable(motor);
    (void)foc_set_torque(motor, 0.5F);
    for (int k = 0; k < kTicks; ++k) {
        (void)foc_tick(motor, kDtS);
    }

    // The table is the contract: a host asks how many registers there are and reads them by position, so a register
    // added later reaches it without a recompile. Hard-coding ten would make the eleventh invisible.
    // 这张表就是契约：宿主问"有多少项"并按位置读，于是以后新增的寄存器不需要它重编就能到达它。把十写死会让第十一项隐形。
    const std::uint32_t count = foc_reg_count();
    std::printf("registers: %u\n", static_cast<unsigned>(count));
    std::printf(" idx        word   read\n");

    std::uint8_t payload[64] = {};
    std::uint32_t filled = 0U;
    for (std::uint32_t i = 0U; (i < count) && (filled + 4U <= sizeof payload); ++i) {
        std::uint32_t word = 0U;
        const foc_err_t got = foc_reg_read(motor, i, &word);
        std::printf(" %3u  0x%08X  %s\n", static_cast<unsigned>(i), static_cast<unsigned>(word),
                    (got == FOC_OK) ? "ok" : "not readable yet");
        put_u32(&payload[filled], word);
        filled += 4U;
    }

    std::uint8_t wire[80] = {};
    const std::uint32_t frame_size = foc_frame_encode(7U, 0x42U, payload, static_cast<std::uint8_t>(filled), wire,
                                                      static_cast<std::uint32_t>(sizeof wire));
    std::printf("\nframe: %u bytes (payload %u)\n", static_cast<unsigned>(frame_size), static_cast<unsigned>(filled));
    if (frame_size == 0U) {
        std::printf("the frame did not fit, and nothing was written\n");
        (void)foc_motor_destroy(motor);
        return 1;
    }

    // The loopback is a memcpy. What is being demonstrated is not the transport -- it is that the register table, the
    // codec and the ABI meet in a C host with no glue of its own.
    // 环回就是一个 memcpy。演示的不是传输——而是寄存器表、编解码器与 ABI 在一个没有任何自备胶水的 C 宿主里接上了。
    std::uint8_t loopback[80] = {};
    for (std::uint32_t i = 0U; i < frame_size; ++i) {
        loopback[i] = wire[i];
    }

    foc_frame_t frame = {};
    const foc_frame_status_t decoded = foc_frame_decode(loopback, frame_size, &frame);
    std::printf("decoded: %s\n", frame_status_name(decoded));
    if (decoded != FOC_FRAME_OK) {
        (void)foc_motor_destroy(motor);
        return 1;
    }

    std::uint32_t mismatches = 0U;
    std::uint32_t offset = 0U;
    for (std::uint32_t i = 0U; (i < count) && (offset + 4U <= frame.length); ++i) {
        const std::uint32_t sent = get_u32(&payload[offset]);
        const std::uint32_t received = get_u32(&frame.payload[offset]);
        if (sent != received) {
            ++mismatches;
        }
        offset += 4U;
    }
    std::printf("seq %u, type 0x%02X, payload %u bytes, %u mismatch(es)\n", static_cast<unsigned>(frame.seq),
                static_cast<unsigned>(frame.type), static_cast<unsigned>(frame.length),
                static_cast<unsigned>(mismatches));

    // Said plainly, because a reader will otherwise wonder: this drive is the library's own stand-in, and its power stage
    // reports no current, so the current words come back as zero. The page is about the round trip being exact, not about
    // the numbers in it -- a real host's numbers come from its own hardware.
    // 直说，否则读者会疑惑：这台驱动器是库自带的替身，它的功率级不报电流，所以电流那几个字回来是零。这一页讲的是往返
    // 精确，不是里面的数字——真实宿主的数字来自它自己的硬件。
    std::printf("note: the stand-in power stage reports no current, so the current words are zero by construction\n");

    // One flipped bit, and the receiver is left exactly as the caller had it: a host that keeps the last good frame
    // keeps a good frame rather than two thirds of a new one.
    // 翻掉一个比特，而接收者保持调用方留下时的样子：保留上一帧的宿主保留的是一帧好帧，而不是新帧的三分之二。
    loopback[frame_size / 2U] = static_cast<std::uint8_t>(loopback[frame_size / 2U] ^ 0x01U);
    foc_frame_t untouched = {0U, 0U, 0U, nullptr};
    const foc_frame_status_t corrupted = foc_frame_decode(loopback, frame_size, &untouched);
    std::printf("one flipped bit: %s; receiver still %s\n", frame_status_name(corrupted),
                (untouched.payload == nullptr) ? "untouched" : "WRITTEN");

    (void)foc_motor_destroy(motor);
    return ((mismatches == 0U) && (corrupted != FOC_FRAME_OK) && (untouched.payload == nullptr)) ? 0 : 1;
}
