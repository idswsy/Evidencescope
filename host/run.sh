#!/usr/bin/env bash
#
# host/run.sh —— 跨平台 cargo 包装
#
# 解决的问题：项目路径含非 ASCII 字符时，Windows + GNU 工具链的 MinGW
# `ld.exe` 打不开带 CJK 的路径，链接阶段会报一堆
# "cannot find ...rcgu.o: No such file or directory"。
#
# 当前 checkout **就是**非 ASCII 路径（D:\比赛\I2C\F103），这段探测会触发；
# 仓库被放到任何中文目录下同样会触发，所以这套探测与绕行必须留着。
#
# 对策：路径含非 ASCII 字符时，自动把 target 目录指到一个纯 ASCII 的位置。
# Linux/macOS/纯 ASCII 路径下完全无副作用。
#
# 用法：
#   ./host/run.sh test
#   ./host/run.sh build --release
#   ./host/run.sh clippy --all-targets
#
set -euo pipefail

cd "$(dirname "$0")"

# ── 探测当前路径是否含非 ASCII 字符 ──────────────────────────────────
if printf '%s' "$PWD" | LC_ALL=C grep -q '[^ -~]'; then
    # 用工号+项目名做隔离，避免不同项目互相踩 target
    export CARGO_TARGET_DIR="${CARGO_TARGET_DIR:-${HOME}/.cargo-target/i2c-scope-f103}"
    echo "注意: 项目路径含非 ASCII 字符，已把 target 目录改到:"
    echo "      $CARGO_TARGET_DIR"
    echo "      （MinGW ld 无法处理 CJK 路径，这是 Windows + GNU 工具链的已知限制）"
    echo
fi

# ── 确保 cargo 在 PATH 里（rustup 装在 ~/.cargo/bin）─────────────────
if ! command -v cargo >/dev/null 2>&1; then
    if [ -x "${HOME}/.cargo/bin/cargo" ]; then
        export PATH="${HOME}/.cargo/bin:${PATH}"
    else
        echo "找不到 cargo。请先安装 Rust：https://rustup.rs" >&2
        exit 1
    fi
fi

exec cargo "$@"
