#!/usr/bin/env bash
# Rebuild the walkthrough, run it, and diff against the output the site was built from.
# 重新编译并运行贯通脚本，与建站时那份输出逐字节比较。
#
# This is the whole test for `guides/walkthrough.rst`: the page does not quote the numbers,
# it includes this program's output, so a change in behaviour that nobody described shows up
# here as a diff instead of quietly becoming documentation.
set -euo pipefail
root=$(cd "$(dirname "$0")/../../.." && pwd)
cd "$root"

g++ -std=c++17 -O2 -Wall -Wextra -Iinclude -Itest_package/test/unit \
    docs/sphinx/scripts/walkthrough.cxx \
    $(find src -name '*.cpp' -o -name '*.c' | grep -v '/version\.') \
    -o /tmp/walkthrough
/tmp/walkthrough > /tmp/walkthrough.out
diff -u docs/sphinx/scripts/walkthrough.expected /tmp/walkthrough.out
echo "walkthrough: output matches docs/sphinx/scripts/walkthrough.expected"
