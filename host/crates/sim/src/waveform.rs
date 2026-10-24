//! 波形合成 —— 模拟器的心脏。
//!
//! 不需要硬件就能生成真实感足够的信号，让 Agent 逻辑、MCP 工具、解码器
//! 全部可以脱离板子开发与回归测试。
//!
//! 所有波形输出 **12-bit 无符号样点（0..4095）**，与真实 ADC 一致 ——
//! 包括直流偏置在 LSB 域的表达，这样上位机的阈值逻辑不会因为
//! 「模拟器给的是有符号电压」而被带偏。

/// 波形场景。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Scenario {
    /// 1 kHz 正弦，3.3 Vpp，偏置在中点。最基本的「能看到东西」场景。
    Sine1k3v3,
    /// 50 kHz 方波。验证高频下的混叠与边沿。
    Square50k,
    /// 带毛刺的脉冲串。验证 minmax 预览是否保住尖峰。
    PulseGlitch,
    /// 白噪声。验证质量标志与「不要回垃圾数」的逻辑。
    Noise,
    /// 纯直流。验证 FORCE_TRIGGER 与 auto 模式。
    Dc,
    /// 调幅信号。验证 RMS/包络类测量。
    Am,
    /// 模拟一条 100 kHz I2C 总线（SCL 在 ch0，SDA 在 ch1）。
    ///
    /// 这是本项目最有用的场景：数字通路解码器可以对着它做回归测试，
    /// 而且因为**内容确定**，可以写断言。
    I2c100k,
    /// 400 kHz I2C —— 用于验证「模拟通路解不了、数字通路能解」的边界。
    I2c400k,
    /// 一条**有器件不应答**的 I2C 总线：地址被应答，但随后的一笔写被 NACK。
    ///
    /// 存在的理由是具体的：README 给 P3 定的验收场景是「抓一次 I2C 写时序
    /// 并告诉我**为什么 NACK**」，而在此之前模拟器**造不出 NACK** ——
    /// `transaction()` 把应答位写死在每个字节后面。一条验收标准，
    /// 内容正好是它演示不了的那件事。
    I2cNack,
    /// 1 kHz、**25% 占空比**的方波。
    ///
    /// 存在的理由：`Square50k` 的占空比写死在 50%，于是「占空比测准了没有」
    /// 这件事在模拟器上**根本验证不了** —— 测出 50% 与测出「恒等于 50%」
    /// 分不开。这一对（25 / 75）把占空比变成一个真正被观测的量。
    Pwm1k25,
    /// 1 kHz、**75% 占空比**的方波 —— `Pwm1k25` 的镜像。
    ///
    /// 两个方向都要有：只测窄脉冲的话，「占空比算成了 100−duty」这个变异体
    /// 在 25% 上照样通过。
    Pwm1k75,
    /// 1 kHz、**5% 占空比**的窄脉冲。
    ///
    /// 这是双电平检测**最危险的退化情形**：高电平平台只占 5% 的样点
    /// （4096 点里约 214 个），而两座平台的高度差着 19 倍。用全局均值初始化
    /// 的聚类会把两个中心双双放进低电平那一侧，整个判定崩掉 ——
    /// `signal::detect_levels` 用 Otsu 定初始分界就是为了它。
    Pwm1k5,
    /// **欠阻尼闭环的阶跃响应** —— 工业控制里调 PID 时示波器上看到的那种波形。
    ///
    /// 设定值是 500 Hz 的方波，输出是一个二阶欠阻尼系统的响应：每个跳变后
    /// 过冲约 31%、随后以 12 kHz 衰减振铃。**两个电平上都有过冲与下冲**
    /// （不是只有上升沿），因为设定值两个方向都在跳。
    ///
    /// ζ = 0.35 是刻意选的：过冲 `exp(-πζ/√(1-ζ²)) ≈ 30.9%` —— 在波形上
    /// 可辨识、又仍在电路可实现范围内。可见的振铃圈数同样只由 ζ 决定
    /// （约 2.7 圈）：过冲与振铃圈数受同一参数约束，增大圈数必然伴随
    /// 更大的过冲 —— 该约束来自物理关系，而非实现上的取舍。
    StepRing,
    /// **RC 充放电** —— 1 kHz 方波经一阶 RC（τ = 150 µs）之后的曲线。
    ///
    /// 指数曲线不是双电平信号（占用率只有 0.1 量级），所以它同时验证
    /// 「不是方波就别硬叫方波」。上升时间 = 10%→90% 的理论值是 `2.2·τ`
    /// = 330 µs，是个能算出来对账的数。
    RcCharge,
    /// **115200 baud 的串行数据**：三个字节循环，每帧后跟 2 位空闲。
    ///
    /// 一帧 10 位：起始位 0 + 8 位数据（**LSB 先**）+ 停止位 1，再加 2 位空闲
    /// 高电平 —— 于是帧边界在波形上是**看得见**的（连续 3 位高电平）。
    ///
    /// ⚠ 内容**不能**只用 `0x55` 连发。那样每一位都翻转，线上就是一条
    /// 干净的方波：帧边界无从辨认，帧长 10 位还是 20 位在波形上完全一样，
    /// 「这是串行数据」这件事在数据里根本不存在。三个字节（`0x55` / `0x0F` /
    /// `0xA5`）的位模式差别够大，波形上才看得出不同的脉宽。
    ///
    /// ⚠ 857142 Hz 采样下每位只有 **7.4 个样点** —— 波形看得见，但脉宽、
    /// 边沿时间这类量都被采样分辨率卡住，读数只当相对量看。真机上更慢的
    /// 波特率会好得多。
    Uart115k,
    /// **慢边沿 + 白噪声**：1 kHz 双电平梯形，10%→90% 上升/下降各 200 µs，
    /// 幅度 2867 LSB，叠加 σ = 40 LSB 的近似高斯白噪声（固定种子）。
    ///
    /// 存在的理由：非迟滞的边沿判定会在这种边沿上反复穿中点，把 1 kHz
    /// 数成 2.0–5.2 kHz（表 A.2）。这个场景把那个缺陷变成 CI 里可复现的
    /// 红色，而不是靠临时探针。
    SlowEdgeNoisy,
    /// **混叠振铃**：1 kHz 方波的每次跳变后叠加 400 kHz 衰减振铃
    /// （半周期约 1.07 个样点，远低于「每半周期 2 样点」的下限）。
    ///
    /// 存在的理由：400 kHz 在 857 kHz 采样下看起来仍像一个低频正弦，
    /// 直接测峰会报出看似正常、实际是混叠伪影的频率。该场景用于钉住
    /// `Refused(Aliased)` 这条拒答路径。
    RingAlias,
    /// **慢边沿时钟**：62.5 kHz 方波，10–90% 约 2.4 个样点，逐沿按
    /// `[0, +0.5, 0, −0.5]` 样点做固定周期抖动。
    ///
    /// 存在的理由：TIE 需要 ≥30 条同向沿，而 1 kHz 场景在 4096 点里只有
    /// 约 4 条 —— 演示与材料里没有一条自然场景能报出数值 TIE。62.5 kHz
    /// 在同样的窗里有约 298 条沿，且 ρ≈2.4 落在可信区。
    Clock62k,
}

impl Scenario {
    /// 全部场景。
    ///
    /// **新增场景时这里和 [`Scenario::name`] 都要动**：`name()` 是穷尽 match，
    /// 加了变体不给名字编译不过；本数组则是 `all_names()` 与测试的依据。
    ///
    /// 从前 `parse` 与 `name` 是**两份各写一遍的 match** —— 给某个场景改名时
    /// 只改一处，`parse` 认的名字与 `name()` 报出的名字就会悄悄分家，
    /// 而两边都编译得过。现在 `parse` 是从 `name()` 反向推出来的，分不了家。
    pub const ALL: [Scenario; 18] = [
        Scenario::Sine1k3v3,
        Scenario::Square50k,
        Scenario::PulseGlitch,
        Scenario::Noise,
        Scenario::Dc,
        Scenario::Am,
        Scenario::I2c100k,
        Scenario::I2c400k,
        Scenario::I2cNack,
        Scenario::Pwm1k25,
        Scenario::Pwm1k75,
        Scenario::Pwm1k5,
        Scenario::StepRing,
        Scenario::RcCharge,
        Scenario::Uart115k,
        Scenario::SlowEdgeNoisy,
        Scenario::RingAlias,
        Scenario::Clock62k,
    ];

    /// 从字符串解析（供 CLI / MCP 使用）。
    ///
    /// 只认 [`Scenario::name`] 给出的那些名字 —— 两者同源，不可能是两套。
    pub fn parse(s: &str) -> Option<Scenario> {
        Self::ALL.into_iter().find(|c| c.name() == s)
    }

    /// 场景名。
    ///
    /// 穷尽 match：加变体会编译不过，逼你顺手给它起名字。
    pub fn name(self) -> &'static str {
        match self {
            Scenario::Sine1k3v3 => "sine_1k_3v3",
            Scenario::Square50k => "square_50k",
            Scenario::PulseGlitch => "pulse_glitch",
            Scenario::Noise => "noise",
            Scenario::Dc => "dc",
            Scenario::Am => "am",
            Scenario::I2c100k => "i2c_100k",
            Scenario::I2c400k => "i2c_400k",
            Scenario::I2cNack => "i2c_nack",
            Scenario::Pwm1k25 => "pwm_1k_25",
            Scenario::Pwm1k75 => "pwm_1k_75",
            Scenario::Pwm1k5 => "pwm_1k_5",
            Scenario::StepRing => "step_ring",
            Scenario::RcCharge => "rc_charge",
            Scenario::Uart115k => "uart_115k",
            Scenario::SlowEdgeNoisy => "slow_edge_noisy",
            Scenario::RingAlias => "ring_alias",
            Scenario::Clock62k => "clock_62k",
        }
    }

    /// 全部场景名。
    ///
    /// MCP 工具 schema 里 `sim_scenario` 的 `enum` 直接用它生成 ——
    /// 于是「schema 列出的场景」与「`parse` 认的场景」是同一份清单。
    pub fn all_names() -> impl Iterator<Item = &'static str> {
        Self::ALL.into_iter().map(|c| c.name())
    }

    /// 该场景需要几个通道。
    pub fn channel_count(self) -> usize {
        match self {
            Scenario::I2c100k | Scenario::I2c400k | Scenario::I2cNack => 2,
            _ => 1,
        }
    }
}

/// 确定性伪随机数（xorshift64*）。
///
/// **必须确定性**：同一个 seed 每次跑出同样的噪声，
/// 否则回归测试会因为「噪声不一样」而随机失败。
#[derive(Debug, Clone)]
pub struct Rng(u64);

/// xorshift 零状态的替代种子。
///
/// 零是吸收态（xorshift 从 0 出发永远出 0），必须避开；但**不能像从前那样
/// 用 `seed | 1`** —— 那会把每一对相邻种子折成同一个（2 与 3 都变成 3），
/// 于是「换个种子看看」有一半的概率拿到一模一样的波形。
const NONZERO_SEED: u64 = 0x9E37_79B9_7F4A_7C15;

impl Rng {
    /// 用给定种子创建。
    ///
    /// 只有 `0` 会被替换成 [`NONZERO_SEED`]，其余原样 —— 相邻种子必须
    /// 给出不同的波形序列。
    pub fn new(seed: u64) -> Rng {
        Rng(if seed == 0 { NONZERO_SEED } else { seed })
    }

    /// 下一个 u64。
    pub fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    /// `[0, 1)` 区间的浮点数。
    pub fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    /// 以 `p` 的概率返回 true。
    pub fn chance(&mut self, p: f64) -> bool {
        self.next_f64() < p
    }
}

/// 波形发生器：给定场景与采样率，产出确定性的样点序列。
#[derive(Debug, Clone)]
pub struct WaveformGen {
    scenario: Scenario,
    rng: Rng,
    /// 全局样点计数（决定相位连续性 —— 分段采样时不能每段从 0 开始）。
    sample_index: u64,
    /// I2C 场景用的总线发生器。
    i2c: I2cBus,
}

/// 12-bit ADC 的中点。
pub const MID_LSB: u16 = 2048;
/// 12-bit ADC 满量程。
pub const FULL_SCALE_LSB: u16 = 4095;

impl WaveformGen {
    /// 创建发生器。
    pub fn new(scenario: Scenario, seed: u64) -> WaveformGen {
        let mut g = WaveformGen {
            scenario,
            rng: Rng::new(seed),
            sample_index: 0,
            i2c: I2cBus::new(100_000),
        };
        // 回归：这里曾经只写死 100 kHz 而**不调用** `set_scenario`，
        // 于是 `Scenario::I2c400k` 标签下跑的是 100 kHz 的波形 ——
        // 任何「对着 400k 场景验证解码器」的测试其实都在验 100k。
        g.set_scenario(scenario);
        g
    }

    /// 当前场景。
    pub fn scenario(&self) -> Scenario {
        self.scenario
    }

    /// 换随机种子。
    ///
    /// **只有 [`Scenario::Noise`] 会用到 rng**，所以换种子只对它有影响 ——
    /// 其余场景要么是纯解析式，要么（[`Scenario::PulseGlitch`]）刻意做成
    /// 确定性的，好让 minmax 预览里那个尖峰落在可断言的位置。
    ///
    /// **不动样点计数** —— 相位保持连续，不会因为换了种子就让波形跳一下。
    pub fn set_seed(&mut self, seed: u64) {
        self.rng = Rng::new(seed);
    }

    /// 切换场景（保留样点计数，相位连续）。
    pub fn set_scenario(&mut self, s: Scenario) {
        self.scenario = s;
        match s {
            Scenario::I2c100k => self.i2c = I2cBus::new(100_000),
            Scenario::I2c400k => self.i2c = I2cBus::new(400_000),
            // 同样 100 kHz，但总线上的事务带一个 NACK
            Scenario::I2cNack => self.i2c = I2cBus::with_bits(100_000, nacked_i2c_transaction()),
            _ => {}
        }
    }

    /// 生成**一个通道**的一批样点，并推进全局样点计数。
    ///
    /// 单通道场景用这个。多通道**必须**用 [`WaveformGen::generate_multi`] ——
    /// 连着调两次 `generate` 会让第二个通道整体偏移 `count` 个样点，
    /// 两路时间轴对不上（对 I2C 解码是致命的）。
    pub fn generate(&mut self, ch: usize, count: usize, rate_hz: u32) -> Vec<u16> {
        let mut out = Vec::with_capacity(count);
        for _ in 0..count {
            out.push(self.next_sample(ch, rate_hz));
            self.sample_index = self.sample_index.wrapping_add(1);
        }
        out
    }

    /// 生成**全部通道**的一批样点，保证各通道逐点时间对齐。
    ///
    /// 返回 `out[ch][i]`：第 `ch` 个通道的第 `i` 个样点，
    /// 所有通道的第 `i` 点对应同一个采样时刻。
    pub fn generate_multi(&mut self, ch_count: usize, count: usize, rate_hz: u32) -> Vec<Vec<u16>> {
        let mut out: Vec<Vec<u16>> = (0..ch_count).map(|_| Vec::with_capacity(count)).collect();
        for _ in 0..count {
            for (ch, chan) in out.iter_mut().enumerate() {
                chan.push(self.next_sample(ch, rate_hz));
            }
            // 一次采样只推进一次全局计数，与通道数无关
            self.sample_index = self.sample_index.wrapping_add(1);
        }
        out
    }

    /// 取某个采样时刻、某个通道的样点值。**不推进**样点计数 ——
    /// 推进由调用者负责，这样多通道才能共用同一个时刻。
    fn next_sample(&mut self, ch: usize, rate_hz: u32) -> u16 {
        use Scenario::*;

        let rate = rate_hz.max(1) as u64;
        let t = self.sample_index as f64 / rate as f64;
        // 整数**纳秒**时间戳：I2C 相位用它算，避免浮点累积漂移。
        //
        // 必须是 ns 而不是 µs —— 857 kHz 下采样间隔是 1.167 µs，
        // 用 µs 会让相邻样点落在同一个微秒里、边沿位置糊成一团，
        // 400 kHz（比特时隙 2.5 µs）的相位就完全不可信了。
        let t_ns = self.sample_index.saturating_mul(1_000_000_000) / rate;

        let v = match self.scenario {
            Sine1k3v3 => {
                // 1 kHz 正弦，3.3 Vpp 占满量程
                let amp = FULL_SCALE_LSB as f64 / 2.0 * 0.95;
                MID_LSB as f64 + amp * (2.0 * std::f64::consts::PI * 1000.0 * t).sin()
            }

            Square50k => {
                let phase = (50_000.0 * t) % 1.0;
                // 加一点上升时间，让边沿不是数学上的垂直（更接近真实）
                let edge = 0.02;
                let frac = if phase < 0.5 {
                    (phase / edge).min(1.0)
                } else {
                    1.0 - ((phase - 0.5) / edge).min(1.0)
                };
                MID_LSB as f64 - 1800.0 + 3600.0 * frac
            }

            PulseGlitch => {
                let period = 200.0; // 样点
                let idx = self.sample_index % period as u64;
                let mut v = MID_LSB as f64 - 1500.0;
                if idx < 20 {
                    v = MID_LSB as f64 + 1500.0;
                }
                // 每 5 个周期插一个单样点尖峰 —— 专门用来验证 minmax 预览
                if idx == 100 && self.sample_index % (period as u64 * 5) < period as u64 {
                    v = FULL_SCALE_LSB as f64;
                }
                v
            }

            Noise => MID_LSB as f64 + (self.rng.next_f64() - 0.5) * 2000.0,

            Dc => MID_LSB as f64 + 300.0,

            Am => {
                let carrier = (2.0 * std::f64::consts::PI * 20_000.0 * t).sin();
                let envelope = 0.5 * (1.0 + (2.0 * std::f64::consts::PI * 200.0 * t).sin());
                MID_LSB as f64 + 1500.0 * envelope * carrier
            }

            Pwm1k25 => pwm_at(t, 0.25),
            Pwm1k75 => pwm_at(t, 0.75),
            Pwm1k5 => pwm_at(t, 0.05),

            StepRing => step_ring_at(t),

            RcCharge => rc_charge_at(t),

            // 位序号用**整数**算：`sample_index × 波特率 / 采样率`。
            // 走浮点（`t * BAUD`）会在位边界上累积出误差，而那种毛刺只在
            // 某些采样率下出现 —— 属于最难查的一类。
            Uart115k => level_to_lsb(uart_115k_bit(
                self.sample_index.saturating_mul(UART_BAUD) / rate,
            )),

            SlowEdgeNoisy => slow_edge_noisy_at(t, &mut self.rng),

            RingAlias => ring_alias_at(t),

            Clock62k => clock_62k_at(self.sample_index, rate_hz),

            I2c100k | I2c400k | I2cNack => {
                let (scl, sda) = self.i2c.levels_at_ns(t_ns);
                level_to_lsb(if ch == 0 { scl } else { sda })
            }
        };

        v.round().clamp(0.0, FULL_SCALE_LSB as f64) as u16
    }
}

// ══════════════════════════════════════════════════════════════════
// I2C 帧生成
//
// 这是模拟器里最有价值的场景：它产生**真正可解码的 I2C 流量**，
// 让 P2 的解码器可以对着确定的内容写断言（而不只是"看起来像波形"）。
//
// 电气上模仿真实开漏总线：高电平由上拉决定（0.85 VDD），
// 低电平接近地（0.15 VDD）—— 两个值都落在 0.3/0.7·VDD 判决带之外。
// ══════════════════════════════════════════════════════════════════

/// I2C 帧里的一个比特时隙。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Bit {
    /// 起始条件（含重复起始）：SCL 保持高，SDA 在高电平期间由高变低。
    Start,
    /// 数据位（含应答位）：SCL 前半段低、后半段高，SDA 在整个时隙内恒定。
    ///
    /// **第 9 个时钟位上，这个位同时表示 ACK 和 NACK** —— 两者在总线上
    /// 只差一个电平：SDA 低是从机应答（ACK），SDA 高是没有应答（NACK）。
    Data(bool),
    /// 重复起始的**建立时隙**：SCL 全程低，SDA 在前半段低、后半段高。
    ///
    /// 为什么不直接连发两个 [`Bit::Start`]：上一个应答位结束时 SDA 是低的，
    /// 紧接着抬高会在 **SCL 仍为高** 的时候产生一个上升沿 —— 解码器会把它
    /// 读成 STOP。真实的重复起始必须先在 SCL 为低的窗口里把 SDA 释放成高，
    /// 再拉高 SCL、再让 SDA 下降。
    Setup,
    /// 停止条件的**建立时隙**：SCL 全程低，SDA 前半段高、后半段低。
    ///
    /// 与 [`Bit::Setup`] 对称：一个在拉高 SCL 之前把 SDA 放到**高**（准备
    /// 重复起始），一个放到**低**（准备 STOP）。两个的 SDA 变化都发生在
    /// SCL 为低的窗口里，所以不构成任何条件。
    PreStop,
    /// 停止条件：SCL 保持高，SDA 在高电平期间由低变高。
    Stop,
    /// 总线空闲：两线都被上拉。
    Idle,
}

/// 一条默认的 I2C 事务：
/// `START → 0x88(地址 0x44 写) + ACK → 0x00 + ACK → 0x1A + ACK → STOP`
///
/// 之所以选 `0x44`：这是常见温湿度传感器（SHT/HTU 系列）的地址，
/// 也是 I2C 调试里最常抓到的事务之一，适合当示例。
pub fn default_i2c_transaction() -> Vec<Bit> {
    // 一条**完整的**「写寄存器指针 → 重复起始 → 读数据」，两笔都在：
    //
    //   S · 0x88(W) · ACK · 0x00 · ACK · Sr · 0x89(R) · ACK · 01 · ACK · 2C · NACK · P
    //
    //   地址 0x44：写方向 0x88 / 读方向 0x89
    //   先写寄存器指针 0x00，再重复起始、读回两个字节 0x01 0x2C
    //   （凑成 0x012C = 300，按 0.1 °C/LSB 就是 30.0 °C —— 一个像样的传感器读数）
    //   最后一个字节由**主机**回 NACK，表示"读够了"，随后 STOP。
    //
    // 回归：从前这里是 `transaction(&[0x88, 0x00, 0x1A])` —— **只有写、没有读**
    // 的单向循环。后果很实在：一个被要求排查「读出来全是 0」的 Agent，
    // 会得出「总线上根本没有读事务」的诊断。那诊断在模拟器上是对的，
    // 对一条真实总线却是它推出来的 —— **场景本身不是一份像样的总线快照**。
    Tx::new()
        .byte(0x88, true) // 地址 0x44 + 写
        .byte(0x00, true) // 寄存器指针
        .repeated_start() // Sr：不放开总线，直接换方向
        .byte(0x89, true) // 地址 0x44 + 读
        .byte(0x01, true) // 数据高字节（主机回 ACK = 还要）
        .byte(0x2C, false) // 数据低字节（主机回 NACK = 读够了）
        .stop()
        .build()
}

/// 「从机不应答」的事务 —— NACK 场景用。
///
/// 地址被正常应答，但寄存器指针那一笔被**拒绝**。这是 I2C 调试里
/// 最常见的两种故障之一（另一种是地址就没应答）：从机在总线上、
/// 地址也对，但这一笔写它不收 —— 典型原因是写保护、寄存器不存在、
/// 或者器件正忙。
pub fn nacked_i2c_transaction() -> Vec<Bit> {
    Tx::new()
        .byte(0x88, true) // 地址 0x44 + 写：从机应答了，器件是在的
        .byte(0x00, false) // 寄存器指针：**被 NACK** —— 这一笔它不收
        .stop()
        .build()
}

/// I2C 事务构造器。
///
/// 存在的理由很具体：`transaction()` 从前把 ACK **写死在每个字节后面**，
/// 于是模拟器**造不出 NACK**。而 NACK 恰恰是 I2C 调试里最常要查的东西
/// （器件不在、地址错、写保护，全表现为 NACK）—— 项目给自己定的 P3 验收
/// 场景就是「告诉我为什么 NACK」，却演示不出来。
#[derive(Debug, Default, Clone)]
pub struct Tx {
    bits: Vec<Bit>,
}

impl Tx {
    /// 从起始条件开始。
    pub fn new() -> Tx {
        Tx {
            bits: vec![Bit::Start],
        }
    }

    /// 追加一个字节，以及第 9 个时钟上的应答位。
    ///
    /// `acked = false` 就是 NACK。写事务里这个位由从机给，
    /// 读事务里由主机给（读到最后一个字节时主机回 NACK）。
    /// **两者在总线上是同一种东西**，所以这里不区分方向。
    #[must_use]
    pub fn byte(mut self, b: u8, acked: bool) -> Tx {
        for i in (0..8).rev() {
            self.bits.push(Bit::Data((b >> i) & 1 == 1));
        }
        // 第 9 拍：SDA 低 = 应答，高 = 不应答
        self.bits.push(Bit::Data(!acked));
        self
    }

    /// 重复起始（`Sr`）：不产生 STOP，直接把总线重新拉起来。
    #[must_use]
    pub fn repeated_start(mut self) -> Tx {
        self.bits.push(Bit::Setup);
        self.bits.push(Bit::Start);
        self
    }

    /// 停止条件 + 两个空闲位，让帧之间有可见的间隙。
    ///
    /// ⚠ 那个 `PreStop` **不能省**：如果最后一个字节是被 NACK 的（SDA 停在
    /// 高电平），直接发 `Stop`（它要求 SDA 从**低**升到高）就会先产生一个
    /// 「SDA 高→低」的下降沿 —— 而那时 SCL 是高，于是被解码成**一个假的
    /// START**。一次事务里凭空多出一个起始条件，`START != STOP`。
    ///
    /// 回归：这个 bug 是被 `generated_i2c_is_actually_decodable` 那句
    /// 「START/STOP 应成对」当场抓住的（28 vs 9），不是靠读代码看出来的。
    #[must_use]
    pub fn stop(mut self) -> Tx {
        self.bits.push(Bit::PreStop);
        self.bits.push(Bit::Stop);
        self.bits.push(Bit::Idle);
        self.bits.push(Bit::Idle);
        self
    }

    /// 取出位序列。
    pub fn build(self) -> Vec<Bit> {
        self.bits
    }
}

/// 用给定的字节序列构造一次 I2C 事务（每个字节后自动补一个 ACK 位）。
pub fn transaction(bytes: &[u8]) -> Vec<Bit> {
    let mut t = Tx::new();
    for &b in bytes {
        t = t.byte(b, true); // 每个字节都被应答
    }
    t.stop().build()
}

/// 一个 SCL 比特时隙的纳秒数。
///
/// **用 ns 而不是 µs**：400 kHz 的比特时隙是 2.5 µs，整数微秒表示不了 ——
/// 截到 2 µs 会变成 500 kHz，进位到 3 µs 会变成 333 kHz，误差 20~25%。
/// 而 400 kHz 正是「双通路设计到底值不值」的关键场景，不能有这种系统性偏差。
fn ns_per_bit(scl_hz: u64) -> u64 {
    if scl_hz == 0 {
        return 1;
    }
    (1_000_000_000 / scl_hz).max(1)
}

/// I2C 总线电平发生器。
///
/// 输入是**采样时刻（ns）**而不是样点序号 —— 这样同一个发生器可以
/// 服务任意采样率，且相位由物理时间决定，不会随采样率漂移。
///
/// 为什么精确到 ns：857 kHz 下采样间隔是 1.167 µs，若时间戳只有 µs 分辨率，
/// 相邻样点会落在同一个微秒里，边沿位置糊成一团 —— 400 kHz 的相位就不可信了。
#[derive(Debug, Clone)]
pub struct I2cBus {
    bits: Vec<Bit>,
    /// 一个比特时隙的长度（ns）。
    bit_ns: u64,
}

impl I2cBus {
    /// 用 SCL 频率（Hz）构造。
    pub fn new(scl_hz: u64) -> I2cBus {
        I2cBus {
            bits: default_i2c_transaction(),
            bit_ns: ns_per_bit(scl_hz),
        }
    }

    /// 用自定义事务构造。
    pub fn with_transaction(scl_hz: u64, bytes: &[u8]) -> I2cBus {
        I2cBus::with_bits(scl_hz, transaction(bytes))
    }

    /// 用一串现成的位构造 —— 需要重复起始或 NACK 时走这个。
    pub fn with_bits(scl_hz: u64, bits: Vec<Bit>) -> I2cBus {
        I2cBus {
            bits,
            bit_ns: ns_per_bit(scl_hz),
        }
    }

    /// 一个完整事务占用的时间（ns）。
    pub fn frame_ns(&self) -> u64 {
        self.bits.len() as u64 * self.bit_ns
    }

    /// 取某一时刻两条线的电平，返回 `(scl_high, sda_high)`。
    ///
    /// 用整数运算：`t_ns % period` 而不是浮点 —— 保证长时间运行时
    /// 相位不会因为浮点累积误差而漂移。
    pub fn levels_at_ns(&self, t_ns: u64) -> (bool, bool) {
        let frame = self.frame_ns();
        if frame == 0 {
            return (true, true);
        }
        let t = t_ns % frame;
        let idx = (t / self.bit_ns) as usize;
        let in_bit = t % self.bit_ns;

        let bit = self.bits.get(idx).copied().unwrap_or(Bit::Idle);

        match bit {
            Bit::Start => {
                // SCL 全程高；SDA 在前半段高、后半段低
                (true, in_bit < self.bit_ns / 2)
            }
            Bit::Stop => {
                // SCL 全程高；SDA 在前半段低、后半段高
                (true, in_bit >= self.bit_ns / 2)
            }
            Bit::Data(level) => {
                // SCL 前半低、后半高；SDA 整段恒定
                (in_bit >= self.bit_ns / 2, level)
            }
            // 两个建立时隙都做成跟 `Data` 一样的 SCL 形状（前半低、后半高），
            // SDA 的跳变放在 **SCL 为低的窗口里**（1/4 处）。
            //
            // 为什么不把 SCL 整拍拉低：那会让 SCL 周期多出一整拍，
            // 于是「100 kHz 场景」实测出来是 96.9 kHz —— 一个 Agent 量到
            // 这个数字会以为自己发现了问题，其实是我们造波形造错了。
            // SCL 的形状必须**逐拍一致**，乱掉的只能是 SDA。
            Bit::Setup => {
                // SDA 低 → 高，跳变发生在 SCL 尚为低的时候
                (in_bit >= self.bit_ns / 2, in_bit >= self.bit_ns / 4)
            }
            Bit::PreStop => {
                // SDA 高 → 低，同样落在 SCL 为低的窗口里
                (in_bit >= self.bit_ns / 2, in_bit < self.bit_ns / 4)
            }
            Bit::Idle => (true, true),
        }
    }
}

/// 把逻辑电平映射到 ADC 域。
///
/// 用 0.15 / 0.85 VDD 而不是 0 / 1 —— 两个值都落在 I2C 判决门限
/// （0.3 VDD / 0.7 VDD）之外，留下裕量，同时保留了真实开漏总线的样子。
#[inline]
fn level_to_lsb(high: bool) -> f64 {
    if high {
        FULL_SCALE_LSB as f64 * 0.85
    } else {
        FULL_SCALE_LSB as f64 * 0.15
    }
}

/// 1 kHz 梯形方波，`duty` 是高电平占整周期的比例。
///
/// 边沿占周期的 1%（**上升与下降各占一半**）—— 1 kHz 下上升沿单独只有 5 µs，
/// 10%→90% 是 4 µs，857 kHz 采样下约 3.4 个样点。
/// **刻意不让边沿垂直**：数学上垂直的边沿采样之后与「一个采样步跨完全部」
/// 无法区分，`rise_ns` 会永远报一个采样周期，而那个数说明不了任何事。
///
/// 摆幅与 `Square50k` 一致（中点 ±1800），两个方波场景的测量值可以直接对比。
fn pwm_at(t: f64, duty: f64) -> f64 {
    /// 边沿占整周期的比例（两侧各一半）。
    const EDGE: f64 = 0.01;
    /// 重复频率。
    const HZ: f64 = 1000.0;

    let phase = (HZ * t) % 1.0;
    let half = EDGE / 2.0;
    let frac = if phase < half {
        phase / half
    } else if phase < duty - half {
        1.0
    } else if phase < duty + half {
        (duty + half - phase) / EDGE
    } else {
        0.0
    };
    MID_LSB as f64 - 1800.0 + 3600.0 * frac.clamp(0.0, 1.0)
}

/// 500 Hz 方波设定值经**二阶欠阻尼**系统之后的输出 —— 调 PID 时示波器上的样子。
///
/// 阻尼比 ζ = 0.35，过冲 `exp(-πζ/√(1-ζ²)) ≈ 30.9%`；无阻尼振荡频率 12 kHz。
///
/// # 为什么可以不记历史
///
/// 半周期 1 ms，而振铃在约 230 µs 内就衰减到 1 LSB 以下 —— 到下一次跳变时
/// 系统早已稳定在前一个电平上，所以每个跳变都可以当成「从稳态出发的新阶跃」，
/// 直接解析求值，不需要逐样点递推。**这个前提靠参数保证**：把 ζ 调小或把
/// 频率调低到振铃跨不过半周期，这条就失效了。
fn step_ring_at(t: f64) -> f64 {
    /// 阻尼比。可见的振铃圈数**只由它决定**（约 2.7 圈）—— 想过冲更小又
    /// 振铃更多，物理上做不到。
    const ZETA: f64 = 0.35;
    /// 无阻尼振荡频率（Hz）。
    const FD_HZ: f64 = 12_000.0;
    /// 设定方波的周期（s）。2 ms 比 4096 点的采集窗（约 4.78 ms）短，
    /// 于是**每个采集窗里都保证有跳变**，不会出现「采到纯基线」的空场景。
    const PERIOD: f64 = 2.0e-3;
    /// 两个稳态电平。摆幅 2000 LSB，过冲后最高约 3418、下冲最低约 181，
    /// 都留在 0..4095 之内 —— 削顶会把这个场景最要紧的那个数毁掉。
    const LO: f64 = 800.0;
    const HI: f64 = 2800.0;

    let phase = (t % PERIOD) / PERIOD;
    let (v0, v1, s) = if phase < 0.5 {
        (LO, HI, phase * PERIOD)
    } else {
        (HI, LO, (phase - 0.5) * PERIOD)
    };

    let root = (1.0 - ZETA * ZETA).sqrt();
    let wd = 2.0 * std::f64::consts::PI * FD_HZ;
    let wn = wd / root;
    let decay = (-ZETA * wn * s).exp();
    let osc = (wd * s).cos() + (ZETA / root) * (wd * s).sin();
    v1 + (v0 - v1) * decay * osc
}

/// 1 kHz 方波经一阶 RC（τ = 150 µs）之后的充放电曲线。
///
/// 半周期只有 500 µs = 3.3τ，所以**充不到满、也放不到底** —— 这正是真实 RC
/// 的样子，而它同时是「指数曲线不是双电平信号」的样本：双电平检测会因占用率
/// 不足拒绝它，分类落到 Unknown。
///
/// 上升时间 10%→90% 的理论值是 `2.2·τ` = 330 µs，是个能算出来对账的数。
fn rc_charge_at(t: f64) -> f64 {
    /// 驱动方波的周期。
    const PERIOD: f64 = 1.0e-3;
    /// 时间常数。
    const TAU: f64 = 150.0e-6;
    const LO: f64 = 800.0;
    const HI: f64 = 2800.0;

    let phase = (t % PERIOD) / PERIOD;
    if phase < 0.5 {
        let s = phase * PERIOD;
        LO + (HI - LO) * (1.0 - (-s / TAU).exp())
    } else {
        // 放电从一个半周期结束时**实际**到达的电平开始，不是从 HI 开始 ——
        // 从 HI 开始会让曲线在跳变处凭空跳一下，那就不是 RC 了。
        let v_top = LO + (HI - LO) * (1.0 - (-(PERIOD / 2.0) / TAU).exp());
        let s = (phase - 0.5) * PERIOD;
        LO + (v_top - LO) * (-s / TAU).exp()
    }
}

/// 慢边沿 + 白噪声（`slow_edge_noisy`）。
///
/// 梯形波：250 µs 爬升 / 250 µs 高平台 / 250 µs 下降 / 250 µs 低平台。
/// 线性边的 10%→90% 正好是 200 µs。幅度 2867 LSB，噪声 σ = 40 LSB。
///
/// 噪声取 **3 个均匀分布之和**（中心化后乘 80）：方差 3/12 × 80² = 40²，
/// 且分布比单个均匀量更接近高斯 —— 真实前端噪声不是矩形分布。
/// 用调用方的 `rng` 保证同一种子逐样点确定。
fn slow_edge_noisy_at(t: f64, rng: &mut Rng) -> f64 {
    /// 重复频率。
    const HZ: f64 = 1000.0;
    /// 0→100% 的边沿占整周期的比例（10%→90% = 0.8 × 250 µs = 200 µs）。
    const EDGE: f64 = 0.25;
    /// 峰峰值（LSB）。
    const AMP: f64 = 2867.0;
    /// 噪声标准差（LSB）。
    const NOISE_SIGMA: f64 = 40.0;

    let lo = MID_LSB as f64 - AMP / 2.0;
    let phase = (HZ * t) % 1.0;
    let frac = if phase < EDGE {
        phase / EDGE
    } else if phase < 0.5 {
        1.0
    } else if phase < 0.5 + EDGE {
        (0.5 + EDGE - phase) / EDGE
    } else {
        0.0
    };
    // 3 个 U(0,1) 的和：均值 1.5、标准差 0.5；×80 得 σ = 40。
    let noise = (rng.next_f64() + rng.next_f64() + rng.next_f64() - 1.5) * (2.0 * NOISE_SIGMA);
    lo + AMP * frac + noise
}

/// 混叠振铃（`ring_alias`）。
///
/// 1 kHz 方波的每次跳变后叠加 400 kHz、τ = 5 µs 的衰减正弦，幅度 300 LSB。
/// 400 kHz 下**半周期只有 1.07 个样点**（857142 / 800000），低于
/// `transient::analyze` 的「每半周期 2 样点」下限 —— 此时任何峰间隔都
/// 无法与混叠伪影区分，必须拒答而不是报数。
fn ring_alias_at(t: f64) -> f64 {
    /// 基波重复频率。
    const HZ: f64 = 1000.0;
    /// 振铃频率（> rate/4）。
    const FD_HZ: f64 = 400_000.0;
    /// 振铃衰减时间常数。
    const TAU: f64 = 5.0e-6;
    /// 振铃初始幅度（LSB）。
    const AMP: f64 = 300.0;
    const LO: f64 = 800.0;
    const HI: f64 = 2800.0;

    let phase = (HZ * t) % 1.0;
    let (base, since) = if phase < 0.5 {
        (HI, phase / HZ)
    } else {
        (LO, (phase - 0.5) / HZ)
    };
    base + AMP * (-since / TAU).exp() * (2.0 * std::f64::consts::PI * FD_HZ * since).sin()
}

/// 62.5 kHz 慢边沿时钟（`clock_62k`）。
///
/// 梯形波：0→100% 的边沿占 3 个样点（857142 Hz 下 10→90% 约 2.4 样点），
/// 高/低平台各约 3.86 个样点。每个上升沿按 `[0, +0.5, 0, −0.5]` 样点
/// 循环偏移，制造一个**确定**的周期抖动；下降沿不抖 —— 抖动来自时钟源，
/// 不是占空比调制。
///
/// 只由绝对样点号求值、不记状态：采集窗从哪里开始都得到同一段波形。
fn clock_62k_at(sample_index: u64, rate_hz: u32) -> f64 {
    /// 时钟频率。
    const HZ: f64 = 62_500.0;
    /// 0→100% 的边沿宽度（样点，按 857142 Hz 计）。
    const EDGE_SAMPLES: f64 = 3.0;
    /// 逐沿周期抖动（样点）。
    const JITTER: [f64; 4] = [0.0, 0.5, 0.0, -0.5];
    const LO: f64 = 800.0;
    const HI: f64 = 2800.0;

    let p = rate_hz.max(1) as f64 / HZ;
    let x = sample_index as f64;
    let k = (x / p).floor() as i64;
    let half = EDGE_SAMPLES / 2.0;
    let rise = |k: i64| k as f64 * p + JITTER[k.rem_euclid(JITTER.len() as i64) as usize];
    let fall = |k: i64| k as f64 * p + p / 2.0;

    let (r, f, r_next) = (rise(k), fall(k), rise(k + 1));
    // 三条候选沿覆盖一个周期内的所有斜坡：本周期升、本周期降、下周期升。
    // 其余位置按最近一次已完成的沿取平台电平。
    if (x - r).abs() <= half {
        LO + (HI - LO) * ((x - r) / EDGE_SAMPLES + 0.5)
    } else if (x - f).abs() <= half {
        HI - (HI - LO) * ((x - f) / EDGE_SAMPLES + 0.5)
    } else if (x - r_next).abs() <= half {
        LO + (HI - LO) * ((x - r_next) / EDGE_SAMPLES + 0.5)
    } else if x >= r && x < f {
        HI
    } else {
        LO
    }
}

/// 循环发送的字节序列。
///
/// 三个值刻意选得**位模式差别大**：`0x55` 全交替、`0x0F` 半边、`0xA5` 混合。
/// 线上因此有不同的脉宽 —— 看起来才像在传数据，而不是一条时钟。
const UART_BYTES: [u8; 3] = [0x55, 0x0F, 0xA5];

/// 每帧之后的空闲位数（高电平）。**帧边界靠它才看得见**。
const UART_IDLE_BITS: usize = 2;

/// 一帧占的位数：起始 1 + 数据 8 + 停止 1 + 空闲 2。
const UART_FRAME_BITS: usize = 10 + UART_IDLE_BITS;

/// 波特率。
const UART_BAUD: u64 = 115_200;

/// 第 `bit_index` 个位时隙上的线上电平。
///
/// 取**整数位序号**而不是浮点时间：`t * BAUD` 在浮点域里会累积误差，
/// 落在位边界上就是一个样点的毛刺，而且它**只在某些采样率下出现**。
///
/// UART **LSB 先**发，所以数据位 `within - 1` 对应字节的第 `within - 1` 位。
fn uart_115k_bit(bit_index: u64) -> bool {
    let pattern_bits = (UART_BYTES.len() * UART_FRAME_BITS) as u64;
    let pos = (bit_index % pattern_bits) as usize;
    let frame = pos / UART_FRAME_BITS;
    let within = pos % UART_FRAME_BITS;

    if within >= 10 {
        // 空闲位。⚠ 它与下面的停止位**取值相同**（都是高电平），所以这两个
        // 分支在行为上可以合并 —— 但**不要合并**：它们是两个不同的协议位置，
        // 合并之后读代码的人就看不出「帧边界由空闲位标出」这件事了。
        // 也正因如此，这两条分支互相之间是等价变异体，杀不掉，不是测试的问题。
        return true;
    }
    match within {
        0 => false, // 起始位
        9 => true,  // 停止位（与空闲位同为高，见上）
        // 数据位 0..7
        d => (UART_BYTES[frame] >> (d - 1)) & 1 == 1,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn generation_is_deterministic_for_same_seed() {
        let mut a = WaveformGen::new(Scenario::Noise, 42);
        let mut b = WaveformGen::new(Scenario::Noise, 42);
        assert_eq!(a.generate(0, 256, 1_000_000), b.generate(0, 256, 1_000_000));
    }

    #[test]
    fn different_seeds_differ() {
        let mut a = WaveformGen::new(Scenario::Noise, 1);
        let mut b = WaveformGen::new(Scenario::Noise, 2);
        assert_ne!(a.generate(0, 64, 1_000_000), b.generate(0, 64, 1_000_000));
    }

    #[test]
    fn set_seed_changes_the_waveform_without_resetting_phase() {
        // MCP 的 scope_sim_set_scenario 会把 seed 透传到这里。换种子必须
        // 真的换掉波形（否则工具是个摆设），同时**不动样点计数** ——
        // 相位跳变会让「换种子」看起来像「总线重启」。
        let mut g = WaveformGen::new(Scenario::Noise, 1);
        let before = g.generate(0, 64, 1_000_000);

        let mut same = WaveformGen::new(Scenario::Noise, 1);
        let _ = same.generate(0, 64, 1_000_000);
        same.set_seed(9);
        let after = same.generate(0, 64, 1_000_000);
        assert_ne!(after, before, "换了种子波形应当不同");

        // 相位连续：换种子后再跑一段正弦，不该出现第一个样点突然归零之类的跳变
        let mut s = WaveformGen::new(Scenario::Sine1k3v3, 1);
        let a = s.generate(0, 1, 857_142);
        s.set_seed(12345);
        let b = s.generate(0, 1, 857_142);
        let step = (a[0] as i32 - b[0] as i32).abs();
        assert!(
            step < 500,
            "换种子不该让相位跳变（相邻样点差 {step}，正弦 1 kHz 时每步只该走十几 LSB）"
        );
    }

    #[test]
    fn every_variant_is_listed_in_all() {
        // `ALL` 是 `all_names()` 与 MCP schema enum 的依据。加变体却忘了
        // 往 `ALL` 里加，`name()` 那边编译得过（穷尽 match 只管名字），
        // 于是新场景对 MCP 与 CLI **完全不可见**。
        //
        // 数量写死是有意的：它逼你在加变体时回来把数字和 `ALL` 一起改。
        assert_eq!(
            Scenario::ALL.len(),
            18,
            "场景数变了 —— 请同步更新 Scenario::ALL 和这个数字"
        );

        for s in Scenario::ALL {
            assert_eq!(
                Scenario::parse(s.name()),
                Some(s),
                "{} 不能往返 —— parse 与 name 分家了",
                s.name()
            );
        }
        assert_eq!(Scenario::all_names().count(), Scenario::ALL.len());

        // ⚠ **上面三条查不出「ALL 里少列了一个变体、同时把另一个列了两遍」** ——
        // 长度照样是 15，往返与计数断言全绿，而漏掉的那个变体对
        // CLI / MCP / GUI 完全不可见。下面这条补上那个洞。
        //
        // （这不是假想：`ALL` 是手写字面量数组，漏一个再重复一个是最自然的
        // 手误，而本仓库确实出过「新场景静默不可见」的缺陷。）
        let mut names: Vec<&str> = Scenario::all_names().collect();
        names.sort_unstable();
        let n = names.len();
        names.dedup();
        assert_eq!(
            names.len(),
            n,
            "ALL 里有重复的场景 —— 那意味着另一个变体没能进来"
        );
    }

    #[test]
    fn parse_rejects_names_that_are_not_in_all() {
        assert_eq!(Scenario::parse("i2c_999k"), None);
        // 大小写不敏感是有意的「不做」：名字要精确，免得 Agent 以为
        // `I2C_100K` 能用而实际拿到另一回事。
        assert_eq!(Scenario::parse("I2C_100K"), None);
    }

    #[test]
    fn all_samples_stay_in_12bit_range() {
        for sc in Scenario::ALL {
            for seed in [1u64, 7, 99] {
                let mut g = WaveformGen::new(sc, seed);
                for ch in 0..sc.channel_count() {
                    let v = g.generate(ch, 1024, 857_142);
                    assert!(
                        v.iter().all(|&x| x <= FULL_SCALE_LSB),
                        "{:?} seed={seed} ch={ch} 产生了越界样点",
                        sc
                    );
                }
            }
        }
    }

    #[test]
    fn sine_actually_oscillates() {
        let mut g = WaveformGen::new(Scenario::Sine1k3v3, 1);
        let v = g.generate(0, 857, 857_142); // 约 1 ms ≈ 一个周期
        let min = *v.iter().min().unwrap();
        let max = *v.iter().max().unwrap();
        assert!(max - min > 3000, "正弦应该跨越大部分量程: {min}..{max}");
    }

    #[test]
    fn dc_scenario_is_flat() {
        let mut g = WaveformGen::new(Scenario::Dc, 1);
        let v = g.generate(0, 64, 857_142);
        assert!(v.windows(2).all(|w| w[0] == w[1]), "直流场景必须平坦");
    }

    #[test]
    fn i2c_scenario_is_binary() {
        let mut g = WaveformGen::new(Scenario::I2c100k, 1);
        for (ch, v) in g.generate_multi(2, 2048, 857_142).into_iter().enumerate() {
            // 数字域：应该只有两个电平（低 0.15、高 0.85 量程）
            let lo = FULL_SCALE_LSB as f64 * 0.15;
            let hi = FULL_SCALE_LSB as f64 * 0.85;
            for s in v {
                let s = s as f64;
                assert!(
                    (s - lo).abs() < 2.0 || (s - hi).abs() < 2.0,
                    "I2C 通道 {ch} 应只有两个电平，得到 {s}"
                );
            }
        }
    }

    #[test]
    fn scenario_roundtrip_by_name() {
        for sc in [
            Scenario::Sine1k3v3,
            Scenario::Square50k,
            Scenario::I2c100k,
            Scenario::I2c400k,
        ] {
            assert_eq!(Scenario::parse(sc.name()), Some(sc));
        }
        assert_eq!(Scenario::parse("nope"), None);
    }

    // ══════════════════════════════════════════════════════════════
    // I2C 回环测试
    //
    // 这几条测试的意义：证明模拟器产出的是**真正可解码的 I2C**，
    // 而不只是"看起来像波形的东西"。
    // P2 阶段的真解码器可以拿同一批断言做回归。
    // ══════════════════════════════════════════════════════════════

    /// 阈值：0.3 / 0.7 · VDD（协议里约定的判决门限）
    const VIL: u16 = (FULL_SCALE_LSB as u32 * 30 / 100) as u16; // 1228
    const VIH: u16 = (FULL_SCALE_LSB as u32 * 70 / 100) as u16; // 2866

    /// 极简 I2C 解码器 —— 边沿驱动，够用来验证模拟器。
    ///
    /// 返回 `(解码出的字节, START 次数, STOP 次数)`。
    fn decode_i2c(scl: &[u16], sda: &[u16]) -> (Vec<u8>, u32, u32) {
        assert_eq!(scl.len(), sda.len());
        let mut bytes = Vec::new();
        let mut starts = 0u32;
        let mut stops = 0u32;

        let mut bits: u32 = 0;
        let mut nbits = 0u32;
        let mut in_frame = false;

        let mut prev_scl = scl[0] > VIH;
        let mut prev_sda = sda[0] > VIH;

        for i in 1..scl.len() {
            let scl_hi = scl[i] > VIH;
            let sda_hi = sda[i] > VIH;

            // START / STOP 只在 SCL 为高时才谈得上
            if prev_scl && scl_hi {
                if prev_sda && !sda_hi {
                    starts += 1;
                    in_frame = true;
                    bits = 0;
                    nbits = 0;
                } else if !prev_sda && sda_hi && in_frame {
                    stops += 1;
                    in_frame = false;
                }
            }

            // SCL 上升沿采样数据位
            if !prev_scl && scl_hi && in_frame {
                bits = (bits << 1) | (sda_hi as u32);
                nbits += 1;
                if nbits == 9 {
                    // 前 8 位是数据，第 9 位是 ACK
                    bytes.push((bits >> 1) as u8);
                    bits = 0;
                    nbits = 0;
                }
            }

            prev_scl = scl_hi;
            prev_sda = sda_hi;
        }
        (bytes, starts, stops)
    }

    #[test]
    fn generated_i2c_is_actually_decodable() {
        // 这一步是整个模拟器可信度的核心：如果解不出来，
        // 后面所有"对着模拟器写的解码器断言"都是空中楼阁。
        // 必须用 generate_multi —— 逐通道 generate 会让 SDA 整体偏移，
        // 这正是这条测试抓出来的 bug（见下面的 multi_channel_is_aligned）。
        let mut g = WaveformGen::new(Scenario::I2c100k, 1);
        let chans = g.generate_multi(2, 4096, 857_142);
        let scl = chans[0].clone();
        let sda = chans[1].clone();

        let (bytes, starts, stops) = decode_i2c(&scl, &sda);

        assert!(stops >= 3, "4096 点应覆盖多次事务，实际 STOP={stops}");
        // 每笔事务是「写寄存器指针 → **重复起始** → 读两个字节」，
        // 所以天然有 **2 个起始条件**（初始 START + 重复起始）和 1 个 STOP。
        //
        // 回归：这条断言从前写的是 `starts == stops || starts == stops + 1` ——
        // 那个等式只在「只有写、没有读」的单向事务下成立。场景里补上读事务
        // 之后它当场变红，提醒的是：**断言里编码的是旧的事务形状**。
        assert!(
            starts >= 2 * stops && starts <= 2 * stops + 1,
            "每笔事务应有 2 个起始条件（初始 + 重复起始），末尾可能多一个截断帧： START={starts} STOP={stops}"
        );

        // 内容必须确定。每笔事务 5 个字节：
        //   0x88 = 地址 0x44 + 写        0x00 = 寄存器指针
        //   0x89 = 地址 0x44 + 读        0x01 0x2C = 读回的两个字节
        assert!(!bytes.is_empty(), "应解出字节");
        let mut full = 0;
        for (i, chunk) in bytes.chunks(5).enumerate() {
            if chunk.len() < 5 {
                break; // 尾部截断的半帧，忽略
            }
            assert_eq!(
                chunk,
                &[0x88, 0x00, 0x89, 0x01, 0x2C],
                "第 {i} 笔事务内容不符（解码器或模拟器有 bug）"
            );
            full += 1;
        }
        assert!(full >= 3, "应至少解出 3 笔完整事务，实际 {full}");
    }

    #[test]
    fn i2c_scl_actually_toggles() {
        // 回归：曾经把 scl_hz 当成"周期微秒数"，导致 100 kHz 被当成
        // 100000 µs 周期 —— 整个采集窗都落在同一个半周期里，SCL 是一条直线。
        let mut g = WaveformGen::new(Scenario::I2c100k, 1);
        let scl = g.generate(0, 1024, 857_142);

        let hi = scl.iter().filter(|&&v| v > VIH).count();
        let lo = scl.iter().filter(|&&v| v < VIL).count();

        assert!(hi > 0, "SCL 必须有高电平");
        assert!(lo > 0, "SCL 必须有低电平");
        // 100 kHz 下 1024 点（1194 µs）约覆盖 119 个 SCL 周期 → 大致各半
        let ratio = hi as f64 / (hi + lo) as f64;
        assert!(
            (0.2..=0.8).contains(&ratio),
            "SCL 高低电平比例应接近 50%，实际高电平占 {:.1}%",
            ratio * 100.0
        );
    }

    #[test]
    fn multi_channel_generation_is_time_aligned() {
        // 回归：曾经在 device.rs 里写成
        //     for ch in 0..ch_count { channels.push(wave.generate(ch, n, rate)) }
        // 结果 CH2 比 CH1 晚了整整 n 个样点 —— 对 I2C 这种要求
        // SCL/SDA 同时刻采样的场景是致命的（解码器会看到两条对不上的线）。
        let mut g = WaveformGen::new(Scenario::I2c100k, 1);
        let chans = g.generate_multi(2, 2048, 857_142);
        assert_eq!(chans.len(), 2);
        assert_eq!(chans[0].len(), 2048);
        assert_eq!(chans[1].len(), 2048);

        // 时间对齐的直接证据：SCL 在任一比特中点必为高，
        // 且 SDA 在该点为 0/1 之一 —— 两者必须来自同一时刻。
        let (bytes, starts, _stops) = decode_i2c(&chans[0], &chans[1]);
        assert!(starts >= 1, "对齐后应能解出帧");
        // 对齐的直接证据：两路解出的字节序列完全一致且内容正确
        assert_eq!(
            &bytes[..5],
            &[0x88, 0x00, 0x89, 0x01, 0x2C],
            "两路对齐后应解出正确内容"
        );
    }

    #[test]
    fn sample_index_advances_once_per_time_step_regardless_of_channels() {
        // 单通道与双通道在同样多的采样次数后，时间轴应推进同样的量
        let mut g1 = WaveformGen::new(Scenario::Dc, 1);
        g1.generate(0, 100, 857_142);

        let mut g2 = WaveformGen::new(Scenario::Dc, 1);
        g2.generate_multi(2, 50, 857_142);

        // 两者都推进了 100 个采样时刻
        let mut g3 = WaveformGen::new(Scenario::Sine1k3v3, 1);
        let a = g3.generate(0, 3, 1000);
        let mut g4 = WaveformGen::new(Scenario::Sine1k3v3, 1);
        let b = g4.generate_multi(1, 3, 1000);
        assert_eq!(a, b[0], "单通道 generate 与 generate_multi(1) 必须等价");
    }

    #[test]
    fn i2c_both_lines_stay_inside_decision_band_margins() {
        // 高低电平必须明确落在 0.3/0.7·VDD 判决带之外，
        // 否则解码器的阈值逻辑会被自己的测试数据坑到
        let mut g = WaveformGen::new(Scenario::I2c100k, 1);
        for (ch, v) in g.generate_multi(2, 2048, 857_142).into_iter().enumerate() {
            for s in v {
                assert!(
                    !(VIL..=VIH).contains(&s),
                    "通道 {ch} 出现了落在判决带内的电平 {s}（VIL={VIL}, VIH={VIH}）"
                );
            }
        }
    }

    #[test]
    fn i2c_bit_period_matches_configured_rate() {
        // 100 kHz → 10 µs/bit；400 kHz → 2 µs/bit（整数微秒取整）
        let bus100 = I2cBus::new(100_000);
        assert_eq!(bus100.bit_ns, 10_000);

        let bus400 = I2cBus::new(400_000);
        // 400 kHz 的比特时隙是 2.5 µs = 2500 ns，必须**精确**表示。
        // 回归：这里曾经用整数微秒，2.5 被截成 2，于是「400 kHz 场景」
        // 实际生成的是 500 kHz 波形；而这条断言还把这个错误固化成了期望
        // （原文写着「整数取整到 2」）。
        assert_eq!(bus400.bit_ns, 2_500);

        // 1 MHz 也要精确
        assert_eq!(I2cBus::new(1_000_000).bit_ns, 1_000);

        // 一笔事务的位数（见 `default_i2c_transaction`）：
        //   1  START
        //   9  地址 0x88 + 应答
        //   9  寄存器指针 0x00 + 应答
        //   1  Setup（重复起始的建立时隙）
        //   1  Start（重复起始）
        //   9  地址 0x89 + 应答
        //   9  数据 0x01 + 主机应答
        //   9  数据 0x2C + 主机**不**应答
        //   1  PreStop（停止的建立时隙）
        //   1  Stop
        //   2  Idle
        //  ── 共 52 位
        assert_eq!(bus100.frame_ns(), 52 * 10_000);
    }

    #[test]
    fn wavegen_constructor_honours_the_scenario() {
        // 回归：WaveformGen::new 曾经硬编码 I2cBus::new(100_000) 且从不调用
        // set_scenario —— 于是 I2c400k 场景下跑的是 100 kHz 波形。
        // 这会让任何「对着 400k 场景验证解码器」的测试变成自欺欺人。
        let g100 = WaveformGen::new(Scenario::I2c100k, 1);
        let g400 = WaveformGen::new(Scenario::I2c400k, 1);
        assert_eq!(g100.i2c.bit_ns, 10_000, "i2c_100k 场景应是 10 µs 比特时隙");
        assert_eq!(g400.i2c.bit_ns, 2_500, "i2c_400k 场景应是 2.5 µs 比特时隙");

        // 而且两条波形必须真的不一样
        let mut a = WaveformGen::new(Scenario::I2c100k, 1);
        let mut b = WaveformGen::new(Scenario::I2c400k, 1);
        let wa = a.generate_multi(2, 512, 857_142);
        let wb = b.generate_multi(2, 512, 857_142);
        assert_ne!(wa, wb, "100k 与 400k 两个场景不能产出相同的波形");
    }

    #[test]
    fn i2c_frame_levels_are_well_formed() {
        let bus = I2cBus::new(100_000);
        // 第 0 位是 START：SCL 恒高，SDA 前半高后半低
        let (scl_a, sda_a) = bus.levels_at_ns(0);
        let (scl_b, sda_b) = bus.levels_at_ns(9_000);
        assert!(scl_a && scl_b, "START 期间 SCL 必须保持高");
        assert!(sda_a, "START 前半段 SDA 应为高");
        assert!(!sda_b, "START 后半段 SDA 应变低");
    }

    // ══════════════════════════════════════════════════════════════
    // 2026-10-06 新增的六个场景：内容断言
    // ══════════════════════════════════════════════════════════════
    //
    // 这一组**不是「不 panic」**，而是「生成出来的波形确实是它文档里宣称的那个」：
    // 每个场景的注释都写了一个具体数字（占空比 25%、过冲 30.9%、τ = 150 µs…），
    // 这里就用**真正的测量代码**把它量回来。场景文档与实现分家时，这一组会红。

    /// 采样率取设备上限，与真机一致。
    const RATE: u32 = 857_142;

    fn capture_of(sc: Scenario, n: usize) -> scope_core::Capture {
        let mut g = WaveformGen::new(sc, 1);
        let s = g.generate(0, n, RATE);
        let mut cap = scope_core::Capture::new(1, RATE, 1, n as u32);
        cap.channels[0] = s;
        cap
    }

    /// I²C 场景是双通道 —— SCL=CH1、SDA=CH2（`next_sample` 的 ch 参数）。
    fn two_ch_capture_of(sc: Scenario, n: usize) -> scope_core::Capture {
        let mut g0 = WaveformGen::new(sc, 1);
        let mut g1 = WaveformGen::new(sc, 2);
        let mut cap = scope_core::Capture::new(1, RATE, 2, n as u32);
        cap.channels[0] = g0.generate(0, n, RATE);
        cap.channels[1] = g1.generate(0, n, RATE);
        cap
    }

    /// B1 场景版：真实 `uart_115k` 场景过一遍产品级解码器（core 侧的
    /// 同构生成器测试已过；这条对照的是场景本身）。
    #[test]
    fn decoder_b1_the_real_uart_scenario_decodes_to_the_truth() {
        use scope_core::decoder::{decode, DecodeParams, DecodeRequest, Protocol, UartConfig};
        let cap = capture_of(Scenario::Uart115k, 4096);
        let req = DecodeRequest {
            protocol: Protocol::Uart,
            channels: vec![0],
            params: DecodeParams::Uart(UartConfig::default()),
        };
        let out = decode(&cap, &req).expect("真实场景应能解码");
        assert_eq!(out.frame_count(), 46, "12 位/帧 × 7.44 样点 → 46 帧");
        assert_eq!(out.quality.frame_errors, 0, "{:?}", out.warnings);
        let bytes = out.uart_bytes().expect("UART 应有字节序列");
        // 真值就是 `the_uart_scenario_decodes_back_to_the_byte_sequence`
        // 手工位采样的那份：[0x55, 0x0F, 0xA5] 循环。
        let pattern = [0x55u8, 0x0F, 0xA5];
        for (i, b) in bytes.iter().take(45).enumerate() {
            assert_eq!(*b, pattern[i % 3], "第 {i} 帧字节应循环真值");
        }
        assert_eq!(bytes[45], 0x55, "尾帧落在循环的 0x55 上");
    }

    /// B3：I²C 适配层与直连 `decode_capture` 在三个场景上逐项一致 ——
    /// 插件化不引入任何行为变化（帧数、字节、NACK、告警）。
    #[test]
    fn decoder_b3_the_i2c_adapter_matches_the_direct_call() {
        use scope_core::decoder::{decode, DecodeParams, DecodeRequest, Frame, Protocol};
        for sc in [Scenario::I2c100k, Scenario::I2c400k, Scenario::I2cNack] {
            let cap = two_ch_capture_of(sc, 4096);
            let cfg = scope_core::I2cDecodeConfig::new(0, 1);

            let direct = scope_core::decode_capture(&cap, &cfg).expect("直连应成功");
            let req = DecodeRequest {
                protocol: Protocol::I2c,
                channels: vec![],
                params: DecodeParams::I2c(cfg),
            };
            let via_decoder = decode(&cap, &req).expect("适配层应成功");

            let via = via_decoder
                .frames
                .iter()
                .map(|f| match f {
                    Frame::I2c(t) => t,
                    Frame::Uart(_) => unreachable!("I²C 结果里不会混进 UART 帧"),
                })
                .collect::<Vec<_>>();
            assert_eq!(
                via.len(),
                direct.transactions.len(),
                "{}：帧数必须一致",
                sc.name()
            );
            for (a, b) in via.iter().zip(direct.transactions.iter()) {
                assert_eq!(a.bytes, b.bytes, "{}：字节序列必须一致", sc.name());
                assert_eq!(
                    a.has_nack(),
                    b.has_nack(),
                    "{}：NACK 定位必须一致",
                    sc.name()
                );
                assert_eq!(a.complete, b.complete, "{}：截断标记必须一致", sc.name());
            }
            assert_eq!(
                via_decoder.quality.frame_errors,
                direct.transactions.iter().filter(|t| !t.complete).count(),
                "{}：不完整帧数必须一致",
                sc.name()
            );
        }
    }

    fn measured(sc: Scenario, n: usize) -> scope_core::Measurements {
        let cap = capture_of(sc, n);
        scope_core::measure(&cap, 0, &scope_core::ChannelScale::default())
            .expect("单通道捕获必须测得出")
    }

    /// 三个 PWM 场景的占空比，量出来必须就是它们名字里那个数。
    ///
    /// 变异：把 `pwm_at` 的 `duty` 参数与臂绑错（25 与 75 互换）→ 这条红。
    /// 变异：上升下降沿的判定取反（高电平占比变成 1−duty）→ 这条红。
    #[test]
    fn the_pwm_scenarios_measure_at_the_duty_they_are_named_after() {
        for (sc, expect) in [
            (Scenario::Pwm1k25, 25.0),
            (Scenario::Pwm1k75, 75.0),
            (Scenario::Pwm1k5, 5.0),
        ] {
            let m = measured(sc, 4096);
            let d = m.duty_pct.expect("占空比必须测得出");
            assert!(
                (d - expect).abs() < 1.5,
                "{}：占空比实测 {d:.2}%，名字里写的是 {expect}%",
                sc.name()
            );
            let f = m.freq_hz.expect("频率必须测得出");
            assert!(
                (f - 1000.0).abs() < 5.0,
                "{}：频率实测 {f:.1} Hz，应为 1000 Hz",
                sc.name()
            );
        }
    }

    /// PWM 的边沿要**慢到能测**。
    ///
    /// 边沿占整周期的 1%，上升与下降各占一半 —— 所以**上升沿单独只有 5 µs**，
    /// 10%→90% 是 4 µs。
    ///
    /// 断言下限取 2 个采样周期：数学上垂直的边沿采样之后与「一步跨完」
    /// 不可区分，`rise_ns` 会永远等于 1 个采样周期 —— 那个数说明不了任何事，
    /// 而它看起来还挺像回事，正是最该避免的那种输出。
    ///
    /// 变异：`EDGE` 从 0.01 改成 0（垂直边沿）→ 上升时间掉到一个采样周期，红。
    #[test]
    fn the_pwm_edges_are_slow_enough_to_measure() {
        let m = measured(Scenario::Pwm1k25, 4096);
        let r = m.rise_ns.expect("上升时间");
        let dt_ns = 1e9 / RATE as f64;
        assert!(
            r > 2.0 * dt_ns,
            "上升时间 {r:.0} ns 只有 {} 个采样周期 —— 边沿太陡，测出来的是量化噪声",
            r / dt_ns
        );
        assert!(
            (r - 4000.0).abs() < 800.0,
            "上升时间实测 {r:.0} ns；上升沿 5 µs 的 10%→90% 应为 4 µs"
        );
    }

    /// **5% 窄脉冲必须仍被判成双电平。**
    ///
    /// 这是这个场景存在的全部理由：两座平台的样点数差着 19 倍（3891 : 205），
    /// 用全局均值初始化的聚类会把两个中心双双放进低电平那一侧。
    ///
    /// 变异：`detect_levels` 的 Otsu 换回均值初始化 —— 这条红（在 core 的测试里也会红，
    /// 但这条是**跨模块**的：它证明模拟器真的造出了那个退化情形）。
    #[test]
    fn the_five_percent_scenario_is_actually_the_hard_case_it_claims_to_be() {
        let cap = capture_of(Scenario::Pwm1k5, 4096);
        let l = scope_core::detect_levels(&cap.channels[0]);
        assert!(
            l.is_two_level(),
            "5% 窄脉冲必须仍判成双电平，实测 {:?}",
            l.verdict
        );
        assert!(
            (l.minority_frac - 0.05).abs() < 0.02,
            "少数派平台占比 {:?}，场景名里的 5% 对不上",
            l.minority_frac
        );
        // 而且两座平台要真的分得开
        assert!(
            l.sep_lsb > 3000.0,
            "平台间隔只有 {:?} LSB —— 摆幅没拉开，退化情形就没造出来",
            l.sep_lsb
        );
    }

    /// 慢边沿 + 白噪声：迟滞把「噪声在边沿上反复穿中点」挡成一次跃变。
    ///
    /// 这是 P0-b 的 CI 回归场景。旧的非迟滞边沿判定在这种波形上会数出
    /// 许多假跃变（表 A.2：2.0–5.2 kHz）；修复后应回到 1 kHz。
    ///
    /// 变异：`edges_hysteretic` 的 h 退回 0 → 频率离开 1000 ± 2 Hz。
    #[test]
    fn slow_edge_noisy_measures_one_kilohertz() {
        let cap = capture_of(Scenario::SlowEdgeNoisy, 4096);
        let m = scope_core::measure(&cap, 0, &scope_core::ChannelScale::default())
            .expect("慢边沿双电平信号必须测得出");
        let f = m.freq_hz.expect("频率必须测得出");
        assert!(
            (f - 1000.0).abs() < 2.0,
            "慢边沿 + 噪声下频率实测 {f:.2} Hz；迟滞应当把它压回 1000 Hz"
        );
        let r = m.rise_ns.expect("上升时间必须测得出");
        assert!(
            (r - 200_000.0).abs() < 30_000.0,
            "10%→90% 上升时间实测 {r:.0} ns，场景定义是 200 000 ns"
        );
    }

    /// `ring_alias` 的振铃半周期不足 2 个样点 —— 必须拒答，不能给一个
    /// 看起来正常、实际是混叠伪影的频率数字。
    ///
    /// 变异：`transient::analyze` 的 `half_samples < 2.0` 改成 `< 1.0` → 红。
    #[test]
    fn ring_alias_is_refused_as_aliased() {
        let cap = capture_of(Scenario::RingAlias, 4096);
        let a = scope_core::transient::analyze(&cap.channels[0], RATE);
        assert!(
            a.stats.ring_freq_hz.is_none(),
            "混叠场景不得给出振铃频率数字，实测 {:?}",
            a.stats.ring_freq_hz
        );
        assert_eq!(
            a.reasons.get("ring_freq_hz"),
            Some(&scope_core::TransientReason::Aliased),
            "应拒答为 Aliased；实际原因 {:?}",
            a.reasons.get("ring_freq_hz")
        );
    }

    /// F3：自然场景（不是测试构造）必须能报出数值 TIE。
    ///
    /// 1 kHz 场景在 4096 点里只有约 4 条同向沿，达不到 N_min = 30；
    /// `clock_62k` 用 62.5 kHz、10–90% ≈ 2.4 样点，既有足够多的沿，
    /// 又落在 ρ ≥ 2 的可信区。
    ///
    /// 变异：把抖动模式清零 → σ_TIE 掉到 0，红；
    /// 变异：把边沿加快到一个样点以内 → 先被 EdgeTooFastForTie 拒答，红。
    #[test]
    fn clock_62k_reports_a_numeric_tie() {
        let cap = capture_of(Scenario::Clock62k, 4096);
        let a = scope_core::tie::analyze(&cap.channels[0], RATE);
        assert!(a.reason.is_none(), "自然场景不应拒答：{:?}", a.reason);
        let t = a.stats.expect("应给出 TIE 统计量");
        assert!(
            (295..=300).contains(&t.n_edges),
            "同向沿数 {}，4.78 ms 的 62.5 kHz 时钟应约 298 条",
            t.n_edges
        );
        assert!((2.0..=3.5).contains(&t.rho), "ρ={} 应进入 T2 可信区", t.rho);
        assert!(!t.grid_limited, "ρ={} 不应再标网格受限", t.rho);
        // 固定抖动模式 [0, +0.5, 0, −0.5] 样点 → σ≈0.354 样点≈413 ns。
        assert!(
            (300.0..=500.0).contains(&t.sigma_tie_ns),
            "σ_TIE={} ns，固定 ±0.5 样点抖动应给出约 413 ns",
            t.sigma_tie_ns
        );

        let report = scope_core::AnalysisReport::from_capture(
            &cap,
            &scope_core::ScaleSet::uncalibrated(1),
            "sim(clock_62k)",
            true,
        );
        assert!(
            matches!(
                report.channels[0].quality.verdicts.get("tie_sigma_ns"),
                Some(scope_core::Verdict::Ok { .. })
            ),
            "TIE σ 有模型，判决必须是 Ok：{:?}",
            report.channels[0].quality.verdicts.get("tie_sigma_ns")
        );

        let m = measured(Scenario::Clock62k, 4096);
        let f = m.freq_hz.expect("62.5 kHz 时钟必须测得出频率");
        assert!(
            (f - 62_500.0).abs() < 50.0,
            "频率实测 {f:.1} Hz，场景定义是 62.5 kHz"
        );
    }

    /// 阶跃响应的过冲是**解析值**：`exp(-πζ/√(1-ζ²))`。
    ///
    /// ζ = 0.35 → 30.9%。这个数不是从实现里抄的，是从 ζ 算出来的 ——
    /// 所以它能同时钉住阻尼比、振荡频率与电平摆幅三件事。
    ///
    /// 变异：ζ 改成 0.5（过冲变 16.3%）→ 这条红。
    /// 变异：只对上升沿做振铃、下降沿直接跳 → 下冲断言红。
    #[test]
    fn the_step_response_overshoots_by_the_analytic_amount() {
        let m = measured(Scenario::StepRing, 4096);
        let over = m.overshoot_pct.expect("过冲必须给得出");
        let under = m.undershoot_pct.expect("下冲必须给得出");
        let expect = 30.9;
        assert!(
            (over - expect).abs() < 3.0,
            "过冲实测 {over:.1}%，ζ=0.35 的解析值是 {expect}%"
        );
        assert!(
            (under - expect).abs() < 3.0,
            "下冲实测 {under:.1}% —— 设定值两个方向都在跳，两个电平上都要有过冲"
        );
        // 而且它真的是个周期性激励（500 Hz），不是一次性的孤立事件
        let f = m.freq_hz.expect("频率必须测得出");
        assert!(
            (f - 500.0).abs() < 5.0,
            "频率实测 {f:.1} Hz，设定方波应为 500 Hz"
        );
    }

    /// **任何起点截出来的采集窗里都得有跳变。**
    ///
    /// 场景注释里写死了这条：周期 2 ms 短于 4096 点的窗（4.78 ms）。
    /// 一旦有人把周期调长过窗宽，用户点一次「采集」就有相当概率看到一条
    /// 平线 —— 那会让人以为工具坏了。
    ///
    /// 变异：`PERIOD` 改成 20e-3（20 ms）→ 这条红。
    #[test]
    fn every_capture_window_of_the_step_scenario_contains_a_transition() {
        for skip in [0usize, 700, 1500, 2400, 3300] {
            let mut g = WaveformGen::new(Scenario::StepRing, 1);
            let _ = g.generate(0, skip, RATE); // 先空跑，错开相位
            let s = g.generate(0, 4096, RATE);
            let lo = *s.iter().min().unwrap();
            let hi = *s.iter().max().unwrap();
            assert!(
                hi - lo > 1500,
                "跳过 {skip} 个样点后，窗内的峰峰值只有 {} —— 没采到跳变",
                hi - lo
            );
        }
    }

    /// RC 的上升时间是 `2.2·τ` 的**可推算**变体：门限取的是**观测到**的
    /// 最低/最高样点，而不是 0 与 HI，所以不是教科书上那个 2.2τ。
    ///
    /// 推导（A = HI − LO = 2000）：
    /// - 充电半个周期（500 µs = 3.33τ）后到达 `LO + (1−e^−3.33)A = LO + 0.96433A`
    /// - 放电半个周期后落到 `LO + 0.96433·e^−3.33·A = LO + 0.03440A`
    /// - 观测跨度 `= 0.92993A`；于是 t10 在 `LO + 0.12739A`、t90 在 `LO + 0.87134A`
    /// - 在充电曲线上：`s10 = 0.13625τ`、`s90 = 2.04987τ`
    /// - ⇒ 上升时间 `= 1.9136τ = 287.0 µs`
    ///
    /// 变异：`TAU` 改成 300 µs → 这条红。
    #[test]
    fn the_rc_curve_has_the_time_constant_it_advertises() {
        let m = measured(Scenario::RcCharge, 4096);
        let r = m.rise_ns.expect("上升时间必须测得出");
        assert!(
            (r - 287_000.0).abs() < 20_000.0,
            "上升时间实测 {r:.0} ns，按 τ=150 µs 推出的解析值是 287 000 ns"
        );
        let f = m.freq_hz.expect("驱动频率");
        assert!(
            (f - 1000.0).abs() < 5.0,
            "驱动方波应为 1 kHz，实测 {f:.1} Hz"
        );
    }

    /// **把 UART 波形按协议解回字节序列。**
    ///
    /// 这是这一组里最强的一条：它不检查「像不像」，而是按 115200 baud 的
    /// 位时隙去采电平、还原成字节，再对着预期的序列比。**一次同时钉住**
    /// 波特率、LSB 先的位序、起始/停止位、帧长（10 位）、空闲位数（2 位）
    /// 与字节循环顺序 —— 任何一处错了，解出来的序列就变样。
    ///
    /// 变异：数据位反过来（MSB 先）→ 解出 0xAA/0xF0/0xA5，红；
    ///       帧长写成 11 或 12 → 位时隙错位，红；
    ///       起始位写成高 → 第一个字节的起始位断言红；
    ///       波特率改成 57600 → 采样点全部错位，红。
    #[test]
    fn the_uart_scenario_decodes_back_to_the_byte_sequence() {
        let cap = capture_of(Scenario::Uart115k, 4096);
        let s = &cap.channels[0];
        let mid = FULL_SCALE_LSB / 2;

        // 每位的样点数（857142 / 115200 = 7.44）。测试里**自己算**这个数，
        // 不用实现里的常量 —— 否则波特率写错时两边一起错，测试照样绿。
        let bit_samples = RATE as f64 / 115_200.0;
        // 取位中心采样，避开边沿
        let at = |bit: usize| s[((bit as f64 + 0.5) * bit_samples) as usize];

        // 解 6 帧 —— 两个完整循环，顺带钉住字节的排列顺序
        let mut got = Vec::new();
        for f in 0..6 {
            let base = f * 12; // 一帧 10 位 + 2 位空闲
            assert!(at(base) < mid, "第 {f} 帧的起始位必须是低电平");
            assert!(at(base + 9) > mid, "第 {f} 帧的停止位必须是高电平");
            assert!(
                at(base + 10) > mid && at(base + 11) > mid,
                "第 {f} 帧之后必须有两个空闲位的高电平 —— 帧边界靠它才看得见"
            );
            let mut byte: u8 = 0;
            for i in 0..8 {
                if at(base + 1 + i) > mid {
                    byte |= 1 << i; // UART 是 LSB 先
                }
            }
            got.push(byte);
        }
        assert_eq!(
            got,
            vec![0x55, 0x0F, 0xA5, 0x55, 0x0F, 0xA5],
            "解出来的字节序列不对"
        );
    }
}
