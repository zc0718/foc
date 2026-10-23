/**
 * @file het_bench_core.h
 * @brief 
 * @author 唐光峰 (2321568810@qq.com)
 * @version 1.0
 * @date 2026-05-26
 * 
 * @copyright Copyright (c) 2026 Inference Engine Team
 * 
 * @par 修改日志:
 * <table>
 * <tr><th>Date       <th>Version <th>Author  <th>Description
 * <tr><td>2026-05-26 <td>1.0     <td>唐光峰     <td>1.首次创建
 * </table>
 */
#ifndef HET_BENCH_CORE_H
#define HET_BENCH_CORE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


typedef int             (*pFunCase)(const void * const ctx);
typedef uint32_t        (*pFunGetTicks)(void);
typedef void            (*pFunLog)(const char *data, uint32_t len);


typedef struct {
        pFunGetTicks                getTicks;
        pFunLog                     write;
} HostInterface;


typedef int             (*pFunEntry)(const HostInterface *host_api);


typedef struct {
        const char                  *name;
        const void                  * const ctx;
        pFunCase                    run;
        uint32_t                    repeat;
} Case;

typedef struct {
        const char                  *moduleName;
        const Case                  *table;
        uint32_t                    caseCount;
} Manifest;


typedef struct {
        uint32_t                    magic;
        uint16_t                    abiMajor;
        uint16_t                    abiMinor;
        const Manifest              *manifest;
        pFunEntry                   entry;
} Benchmark;


void bench_set_host_api(const HostInterface *host_api);
int bench_run_all(const Manifest * const pManifest);

extern int bench_module_entry(const HostInterface *hostApi);


#define       BENCHMARK_CASE_IMPLEMENTATION(name, ctx, run, repeat)             \
        {name, ctx, run, repeat}

#define       BENCHMARK_IMPLEMENTATION(name, cases)                             \
        static const Manifest g_manifest = {                                    \
                .moduleName = name, .table = cases,                             \
                .caseCount = (uint32_t)(sizeof(cases) / sizeof(cases[0])),      \
        };                                                                      \
        int bench_module_entry(const HostInterface *hostApi)                    \
        {                                                                       \
                bench_set_host_api(hostApi);                                    \
                bench_prepare_input();                                          \
                return bench_run_all(&g_manifest);                              \
        }                                                                       \
        __attribute__((section(".algo_header"), used))                          \
        static volatile const Benchmark g_algo_header = {                       \
                .magic = 0x414C474FU, .abiMajor = 1U,                           \
                .abiMinor = 0U, .manifest = &g_manifest,                        \
                .entry = bench_module_entry,                                    \
        }

#ifdef __cplusplus
}
#endif

#endif
