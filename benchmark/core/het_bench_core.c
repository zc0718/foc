/**
 * @file bench_core.c
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
#include "het_bench_core.h"

#include <stddef.h>


static const HostInterface *g_bench_host_api = 0;


static inline uint32_t __get_ticks(void)
{
        return g_bench_host_api && g_bench_host_api->getTicks ? g_bench_host_api->getTicks() : 0U;
}

static inline void __log(const char *data, uint32_t len)
{
        if ((g_bench_host_api != 0) && (g_bench_host_api->write != 0))
        {
                g_bench_host_api->write(data, len);
        }
}


static uint32_t bench_strlen(const char *text)
{
        uint32_t len = 0U;
        if (text == 0)
        {
                return 0U;
        }

        while (text[len] != '\0')
        {
                ++len;
        }
        return len;
}

static void bench_put_u32(uint32_t value)
{
        char digits[10];
        uint32_t i = 0U;

        if (value == 0U)
        {
                char c = '0';
                __log(&c, 1U);
                return;
        }

        while ((value != 0U) && (i < (uint32_t)sizeof(digits)))
        {
                digits[i++] = (char)('0' + (value % 10U));
                value /= 10U;
        }

        while (i > 0U)
        {
                --i;
                __log(&digits[i], 1U);
        }
}

static void __print_manifest(const Manifest * pManifest)
{
    __log("MODULE|", 7U);
    __log(pManifest->moduleName, bench_strlen(pManifest->moduleName));
    __log("|cases=", 7U);
    bench_put_u32(pManifest->caseCount);
    __log("\r\n", 2U);
}

static void __print_result(const char *name, const uint32_t cycles)
{
    __log("RESULT|", 7U);
    __log(name, bench_strlen(name));
    __log("|", 1U);
    bench_put_u32(cycles);
    __log("\r\n", 2U);
}


void bench_set_host_api(const HostInterface * hostApi)
{
        g_bench_host_api = hostApi;
}

int bench_run_all(const Manifest * const pManifest)
{
        int status = 0;
        uint32_t repeat = 0U;
        __log("BENCHMARK_START\r\n", 17U);

        __print_manifest(pManifest);
        for (uint32_t i = 0U; i < pManifest->caseCount; ++i)
        {
                const Case * pCase = &pManifest->table[i];
                if ((pCase->name == 0) || (pCase->run == 0)) {
                        continue;
                }

                const uint32_t loops = (pCase->repeat == 0U) ? 1U : pCase->repeat;
                const uint32_t start = __get_ticks();
                for (repeat = 0U; repeat < loops; ++repeat)
                {
                        status = pCase->run(pCase->ctx);
                        if (!status)    break;
                }
                const uint32_t end = __get_ticks();
                if (!status)
                {
                        __log("CASE_ERROR| ", 12U);
                        __log(pCase->name, bench_strlen(pCase->name));
                        __log(" [", 2U);
                        bench_put_u32(repeat);
                        __log(" / ", 3U);
                        bench_put_u32(loops);
                        __log("]\r\n", 3U);
                }
                __print_result(pCase->name, end - start);
        }

        __log("BENCHMARK_END\r\n", 15U);
        return status;
}
