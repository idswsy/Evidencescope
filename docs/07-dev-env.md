# 07 · 开发环境搭建

> 目标：**新成员克隆仓库后 15 分钟内能跑通 `cargo test` 与 C 端向量测试。**
> 遇到问题先看第 6 节「已知的坑」，那两个坑几乎人人都会踩。

---

## 1. 需要装什么

| 组件 | 版本 | 用途 | 谁要装 |
|---|---|---|---|
| **git** | ≥ 2.30 | 版本管理 | 全员 |
| **Rust** (rustup) | stable | 上位机 | 上位机 + Agent |
| **Python** | ≥ 3.8 | 生成黄金测试向量 | 全员（跑测试要用） |
| **C 编译器** (gcc) | C11 | 协议层测试（`./proto/run_tests.sh`） | **全员** —— 见下方说明 |
| **Keil MDK** | 5.x + **ARMCC5**（工程实测 `V5.06 update 5`；AC6 未验证） | 固件编译 | 固件 |
| **DAP-Link** | — | 烧录 | 固件 |

> **Python 与 gcc 都不是可选项。** `proto/tests/vectors.json` 由 `gen_vectors.py`
> 生成，C 端测试又依赖 `gen_header.py` 把它转成头文件，再交给 gcc 编译 —— 三步缺一不可。
> 协议是三方交汇点，**谁改了协议都要自己先跑一遍两端测试**，所以三个人都需要它们。

### 安装命令

<details>
<summary>Windows</summary>

```powershell
# Rust：用 GNU 工具链，避开 4GB 的 VS Build Tools
winget install Rustlang.Rustup
# 或下载 rustup-init 后：
#   rustup-init.exe -y --default-host x86_64-pc-windows-gnu

# Python
winget install Python.Python.3.12

# C 编译器：MSYS2 或 MinGW-w64
winget install MSYS2.MSYS2      # 然后 pacman -S mingw-w64-x86_64-gcc

# 或者已经装了 Git for Windows，可以用它的 make（如果装了）
```

> **为什么用 GNU 而不是 MSVC 工具链？**
> MSVC 需要 4 GB 的 Visual Studio Build Tools，而本项目只用到
> `serialport` 这类纯 Rust 库，GNU 完全够用。
> **例外**：将来若要做 Tauri GUI，需要换回 MSVC（WebView2 依赖它）。

</details>

<details>
<summary>Linux / macOS</summary>

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
sudo apt install build-essential python3        # Debian/Ubuntu
brew install gcc python3                        # macOS
```

</details>

---

## 2. 跑通验证（10 分钟）

```bash
git clone https://github.com/idswsy/f103-agent-scope.git
cd f103-agent-scope

# ── 1. C 端协议测试 ──────────────────────────────────
./proto/run_tests.sh
# 期望：全部通过: N 项（N 由向量表决定，当前为 50；条数不要写死在别处）

# ── 2. Rust 上位机测试 ───────────────────────────────
./host/run.sh test
# 期望：所有 test result: ok，0 failed
#       **不写死条数** —— 这个数字每加一个测试就过期一次。
#       判断标准只有一个：所有 test result 都是 ok。
#       （2026-10-10 实测：不含 GUI 的 workspace 为 302 项，含 GUI 另 84 项 + 5 ignored）

# ── 3. 端到端：对着模拟器抓一次波形 ──────────────────
./host/run.sh run -p scope-cli -- sim capture --scenario sine_1k_3v3 -n 1024 -o wave.csv
# 期望：打印采集摘要，并写出 wave.csv

# ── 4. MCP 工具链自检 ────────────────────────────────
./host/run.sh run -p scope-mcp -- --selftest
# 期望：自检通过
```

**四条全绿 = 环境就绪。** 没硬件也能到这一步。

---

## 3. 日常命令

| 目的 | 命令 |
|---|---|
| Rust 全部测试 | `./host/run.sh test` |
| C 协议测试 | `./proto/run_tests.sh` |
| 重新生成黄金向量 | `./proto/run_tests.sh --regen` |
| 格式 + 静态检查 | `./host/run.sh fmt --all && ./host/run.sh clippy --all-targets` |
| 列出可用串口 | `./host/run.sh run -p scope-cli -- ports` |
| 连真机抓波形 | `./host/run.sh run -p scope-cli -- serial --port COM3 capture -n 2048 -o wave.csv` |
| 对模拟器开发 | `./host/run.sh run -p scope-cli -- sim capture --scenario i2c_100k -n 4096 -o i2c.csv` |
| UART 单通道解码 | `./host/run.sh run -p scope-cli -- sim decode --protocol uart --scenario uart_115k -n 4096` |
| 打开桌面 GUI | `./host/run.sh run -p scope-gui -- --demo --scenario i2c_100k` |
| 只跑某个 crate | `./host/run.sh test -p scope-mcp` |

> **始终用 `./host/run.sh` 而不是直接 `cargo`** —— 见下面的坑 #1。
>
> ⚠ 它内部会 `cd` 到 `host/`，所以上面那条命令的 `-o wave.csv` 会落在
> **`host/wave.csv`**，不是仓库根目录。要放别处就写相对 `host/` 的路径或绝对路径。

---

## 4. 固件环境（Keil MDK）

1. 安装 **Keil MDK-ARM**（社区版即可）
2. 安装 **STM32F1xx DFP**（Device Family Pack）
3. 打开 `firmware/MDK-ARM/synthesize-project.uvprojx`（**工程已存在**：`App/` +
   `Hardware/` + `Core/` + `proto/protocol.c` 都在里面；最后一次完整重建用的是
   ARMCC `V5.06 update 5`，**0 Error / 0 Warning**）
   > ⚠️ **不要用 `firmware/synthesize-project.ioc` 让 CubeMX 重新生成代码**：它的外设
   > 清单只有 ADC1 / DMA / NVIC / RCC / SPI1 / SYS / TIM2 / TIM3，**既没有 TIM4
   > （ADC 的外部触发）也没有 USART1（协议链路）** —— 重新生成会把这两条链路一起删掉。
   > `Core/Src/tim.c` 的 `MX_TIM4_Init()` 与 `Core/Src/usart.c` 的 `MX_USART1_UART_Init()`
   > 才是当前工程的真实配置。
   > 不想开 Keil 也可以只跑协议逻辑：固件侧那份在 PC 上用 gcc 就能编 —— 直接跑
   > `./proto/run_tests.sh`：它会自己生成向量头、决定编译参数、跑完报数。

想手工编的话**必须带上生成目录** —— `vectors.h` 是生成物，不在 `proto/` 下：

```bash
python proto/tests/gen_header.py proto/tests/vectors.json > proto/build/vectors.h
gcc -std=c11 -Iproto -Iproto/build -o /tmp/tv \
    proto/protocol.c proto/tests/test_vectors.c \
    && /tmp/tv
```

4. 调试器选 **DAP-Link**（ST-Link 在 Win11 上可能与 DAPLink 驱动冲突）

### 烧录方式

| 方式 | 适用 | 说明 |
|---|---|---|
| **DAP-Link / SWD** ⭐ | 日常开发 | 用核心板上的 `3V3/DIO/CLK/GND` 四脚 |
| 外接 USB-TTL 串口 ISP | 没有调试器时 | BOOT0=1、BOOT1=0 进 ROM bootloader |
| USB DFU | ⚠️ **不要指望** | F103C8T6 的系统 bootloader 是否支持原生 USB DFU 存在争议 |

### ⚠️ 数据链路的引脚选择

串口经**外接 CH340 模块**接到 USART1，**需要接四根线**。
详见 [`02-hardware.md`](02-hardware.md) §4 与 [ADR-013](06-decisions.md)。

---

## 5. 编辑器

### VS Code 推荐配置

[`.vscode/settings.json`](../.vscode/settings.json) 已经提交进仓库，**别在这里再抄一份** —— 直接看那个文件。其中只有一项值得解释：

**`rust-analyzer.cargo.extraEnv.CARGO_TARGET_DIR`** 指向 `${env:USERPROFILE}/.cargo-target/...`
—— 理由见坑 #1：`ld.exe` 打不开 CJK 路径。用 `USERPROFILE` 而不是 `HOME`，
是因为 Windows 的 VS Code 里 `HOME` 未必被设置。

### 协议改动的工作流

**改协议 = 改四处 + 一次提交**（这是硬纪律，CI 会卡）：

1. `docs/03-protocol.md`
2. `proto/protocol.h`
3. `host/crates/proto/src/lib.rs`
4. `proto/tests/vectors.json`（跑 `./proto/run_tests.sh --regen`）

然后两端测试必须全绿：

```bash
./proto/run_tests.sh && ./host/run.sh test
```

---

## 6. ⚠️ 已知的坑

### 坑 #1：中文路径导致链接失败（Windows）

**症状**：`cargo build` 在链接阶段报一大堆

```
ld.exe: cannot find C:\Users\...\I2C示波器\F103\host\target\debug\deps\xxx.rcgu.o
ld.exe: cannot find ...\list.def: No such file or directory
collect2.exe: error: ld returned 1 exit status
```

**原因**：MinGW 的 `ld.exe` 用系统 ANSI 代码页解释路径，
而 rustc 传的是 UTF-8 —— 路径里的 `示波器` 三个字直接打不开。

**解决**：把 target 目录指到纯 ASCII 路径。

```bash
export CARGO_TARGET_DIR="C:/cargo-target/i2c-scope"
```

**`./host/run.sh` 会自动检测并处理这件事** —— 这就是它存在的原因。
要一劳永逸也可以写进用户环境变量。

> 把项目移到纯英文路径（如 `C:\dev\i2c-scope`）是更彻底的办法，
> 但当前路径是用户指定的，所以用环境变量绕过。

### 坑 #2：`link.exe` 被 Git Bash 的 coreutils 抢走

**症状**：MSVC 工具链下链接时报奇怪的参数错误。

**原因**：Git Bash 的 `/usr/bin/link`（coreutils 的建立硬链接工具）
在 PATH 里排在 MSVC 的 `link.exe` 前面。

**解决**：本项目用 GNU 工具链，**不会遇到这个问题**。
若将来切 MSVC，把 `/usr/bin` 从 PATH 里挪后，或用完整路径调 cargo。

### 坑 #3：`make` 在 Git Bash 里不存在

**症状**：`make -C proto test` 报 `command not found`。

**解决**：用 `./proto/run_tests.sh`，功能完全等价。
CI（Ubuntu）上 `make` 是可用的，两条路都支持。

### 坑 #4：串口被占用

**症状**：`Failed to open COM3` / `Access is denied`。

**原因**：串口助手、Keil 的调试器、上次没退干净的进程占着。

**解决**：关掉串口助手；Keil 里点 Stop Debug；实在不行拔插一次 USB。

### 坑 #5：USB CDC 连上后设备突然复位

**症状**：刚 `open` 就收到一堆乱码，或者 GET_INFO 无响应。

**原因**：USB CDC 首次打开时 DTR 拉低可能触发板子复位。
PA12 疑似 1.5 kΩ 硬上拉（`【待测】` —— **这是上一块板的结论，换板后待重测**），概率不低。

**解决**：连接后等 200 ms 再发第一条命令；
或在固件里加 DTR 忽略逻辑。

---

## 7. 团队协作建议

| 角色 | 只装这些就够 |
|---|---|
| 上位机 / Agent | git + Rust + Python |
| 固件 | git + Python + Keil + DAP-Link + MinGW（跑 C 端测试）|
| 硬件 | git + Python + 立创EDA专业版 + 万用表 |
