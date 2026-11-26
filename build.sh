#!/bin/bash
set -e

# 过滤 Trae 注入的污染路径（旧版 libstdc++ 会导致 cmake 报 GLIBCXX 错误）
# .bashrc 里也有一份，但 script 模式下 .bashrc 不自动执行
if [[ -n "$LD_LIBRARY_PATH" && "$LD_LIBRARY_PATH" == *"/root/.trae"* ]]; then
    export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v '/root/.trae' | grep -v '^$' | tr '\n' ':')
    export LD_LIBRARY_PATH=${LD_LIBRARY_PATH%:}
fi
export MVS_PATH=/opt/MVS/lib/aarch64/
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

# 确保从项目根目录执行
cd "$(dirname "$0")"

# 自动 cmake（build 目录不存在或 CMakeCache 过期）
if [ ! -d build ] || [ ! -f build/Makefile ] || ! grep -q 'CMAKE_BUILD_TYPE' build/CMakeCache.txt 2>/dev/null; then
    echo "[build.sh] 首次配置，运行 cmake ..."
    mkdir -p build && cd build
    cmake -DCMAKE_BUILD_TYPE=Release ..
    cd ..
fi

cd build
echo "[build.sh] make -j6 ..."
make -j6 "$@"
cd ..

echo "[build.sh] 完成: $(ls -lah build/auto-aim | awk '{print $5, $9}')"
