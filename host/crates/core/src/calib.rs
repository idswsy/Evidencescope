//! 按设备 `uid` 索引的通道标定表。
//!
//! # 为什么需要它
//!
//! 伏特换算**只在上位机做**（ADR-006、`docs/03-protocol.md`），MCU 只认 LSB。
//! 而换算要两个数：每 LSB 对应多少伏、零点落在哪个 LSB。标定之前这两个数
//! 只能猜 —— [`ChannelScale::default`] 给的是「3.3 V / 4096、零点 2048」。
//!
//! [`DeviceInfo::uid`](crate::DeviceInfo::uid) 的注释一直写着「标定表按它索引」，
//! 但那张表不存在。**这个文件就是那张表。**
//!
//! # 边界（本轮刻意不做）
//!
//! - **只支持手工录入。** 引导式标定（短接输入定零点、再给已知参考定增益）
//!   需要真机与一个可信参考源，而那 12 项实板待测项还没做，做了也验不了。
//! - **AI 只读。** 标定会改变之后**所有**的测量值，它是设备状态而不是单次
//!   操作。写入只走 CLI（`scope-cli cal set`），MCP 侧只读出「标没标定」。
//! - **不做插值、不按量程分档。** 底板只有一路模拟输入且量程由机械开关切换
//!   （ADR-010、`docs/02-hardware.md` §5），本轮一条记录一条 `ChannelScale`。
//!
//! # 文件格式
//!
//! 与 AI 配置同目录（`%APPDATA%\scope-gui\calib.json`）——
//! **目录名不要改**，改了会把已有的 `config.json` 撇下，那是最难回滚的一步。
//!
//! ```json
//! { "version": 1,
//!   "devices": { "5eed00010203040506070809": {
//!       "updated_unix": 1780560000, "note": "台架 1.000 V 基准",
//!       "channels": [ { "volts_per_lsb": 8.0566e-4, "zero_lsb": 2048.0 } ] } } }
//! ```
//!
//! 键是 uid 的**规范形式**：小写十六进制、无分隔符、24 字符。写入一律规范化，
//! 读取宽容（接受 `5E:ED:...` 这类写法）。两种拼法会静默产生两条记录，
//! 那是唯一无法事后补救的迁移 bug，所以 [`decode_uid`] 特意宽容而
//! [`encode_uid`] 特意统一。

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use crate::capture::ChannelScale;
use crate::persist;

/// 标定文件的格式版本。
///
/// 读到**更高**版本时只读不写（见 [`CalibrationStore::is_read_only`]）——
/// 免得旧程序把新程序写的字段覆盖没了。
pub const CALIB_VERSION: u32 = 1;

/// 标定文件所在目录名。与 AI 配置共用 —— 见模块头。
const DIR_NAME: &str = "scope-gui";
/// 标定文件名。
const FILE_NAME: &str = "calib.json";

/// 写文件时用来点名「写的是什么」，出现在拒写消息里。
const WRITE_SUBJECT: &str = "标定数据";

/// 一个设备的标定记录。
#[derive(Debug, Clone, Default, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct DeviceCalib {
    /// 写入时刻（Unix 秒）。core 不引时间库，由写方给。
    pub updated_unix: Option<u64>,
    /// 备注，例如「台架 1.000 V 基准」。
    ///
    /// 也是**同一占位 uid 下多块板**的唯一区分手段 —— 见 [`is_placeholder_uid`]。
    pub note: Option<String>,
    /// 下标即通道号；缺的通道视为未标定。
    pub channels: Vec<ChannelScale>,
}

/// 整个标定文件。
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct CalibrationFile {
    /// 格式版本。**故意不给 `#[serde(default)]`** —— 缺版本按损坏处理。
    pub version: u32,
    /// 键是 uid 的规范形式。
    ///
    /// `BTreeMap` 而非 `HashMap`：输出顺序稳定，人读与 diff 都有意义。
    #[serde(default)]
    pub devices: BTreeMap<String, DeviceCalib>,
}

impl Default for CalibrationFile {
    fn default() -> Self {
        CalibrationFile {
            version: CALIB_VERSION,
            devices: BTreeMap::new(),
        }
    }
}

/// 一次采集要用的逐通道换算，外加「它从哪来」。
///
/// 三个「标没标定」的谓词是**单一真相源**：CLI 的提示、GUI 的角标、
/// MCP 响应里的布尔字段全部从这里取，不各自判断。
#[derive(Debug, Clone)]
pub struct ScaleSet {
    uid: Option<[u8; 12]>,
    /// 长度**恒等于**通道数 —— 所以 [`ScaleSet::as_slice`] 可以直接喂给
    /// `Capture::to_csv`，那条「短表就按原始 LSB 输出」的兜底路径永不触发。
    scales: Vec<ChannelScale>,
    /// 与 `scales` 等长：该通道的值是否来自标定记录。
    calibrated: Vec<bool>,
    /// 该 uid **有没有记录**（区别于「记录里有没有这个通道」）。
    record_found: bool,
}

impl ScaleSet {
    /// 全部用占位换算。设备未连接、或没有该 uid 的记录时用它。
    pub fn uncalibrated(ch_count: usize) -> Self {
        ScaleSet {
            uid: None,
            scales: vec![ChannelScale::default(); ch_count],
            calibrated: vec![false; ch_count],
            record_found: false,
        }
    }

    /// 用调用方给定的逐通道换算表构造。**不算标定记录** ——
    /// CSV 导入的换算来自文件 / 命令行参数，与设备标定表无关。
    pub fn from_scales(scales: Vec<ChannelScale>) -> Self {
        Self {
            uid: None,
            calibrated: vec![false; scales.len()],
            scales,
            record_found: false,
        }
    }

    /// 取通道 `ch` 的换算。越界或该通道无记录 → 占位值。
    pub fn get(&self, ch: usize) -> ChannelScale {
        self.scales.get(ch).copied().unwrap_or_default()
    }

    /// 该通道是否来自标定记录。
    pub fn is_calibrated(&self, ch: usize) -> bool {
        self.calibrated.get(ch).copied().unwrap_or(false)
    }

    /// 有没有任何一个通道是标定过的。
    pub fn any_calibrated(&self) -> bool {
        self.calibrated.iter().any(|c| *c)
    }

    /// 是不是每个通道都标定过。**空集返回 `false`** —— 「一个通道都没有」
    /// 不该被读成「全都标定好了」。
    pub fn all_calibrated(&self) -> bool {
        !self.calibrated.is_empty() && self.calibrated.iter().all(|c| *c)
    }

    /// 该 uid 有没有标定记录。
    pub fn record_found(&self) -> bool {
        self.record_found
    }

    /// uid 的规范十六进制形式。
    pub fn uid_hex(&self) -> Option<String> {
        self.uid.as_ref().map(encode_uid)
    }

    /// 逐通道换算表。长度恒等于通道数。
    pub fn as_slice(&self) -> &[ChannelScale] {
        &self.scales
    }

    /// 给人看的一句话，CLI 与 MCP **共用这一份**，免得两处措辞各自漂移。
    pub fn summary_note(&self) -> String {
        if self.all_calibrated() {
            let c = self.get(0);
            return format!(
                "标定：uid {} 的记录（{:.6} V/LSB、零点 {}）",
                self.uid_hex().unwrap_or_else(|| "?".into()),
                c.volts_per_lsb,
                c.zero_lsb
            );
        }
        if self.record_found {
            let missing: Vec<String> = (0..self.scales.len())
                .filter(|i| !self.is_calibrated(*i))
                .map(|i| format!("CH{}", i + 1))
                .collect();
            return format!(
                "⚠ 电压部分未标定：uid {} 的记录里缺 {}，这些通道用占位换算",
                self.uid_hex().unwrap_or_else(|| "?".into()),
                missing.join("、")
            );
        }
        format!(
            "⚠ 电压未标定：未找到该 uid 的标定记录，用占位换算（3.3 V / 4096、零点 2048）。\
             写入：scope-cli cal set --uid {} --ch 0 --volts-per-lsb <V>",
            self.uid_hex().unwrap_or_else(|| "<UID>".into())
        )
    }
}

/// 标定表。持有一份文件内容 + 它从哪读的。
#[derive(Debug, Clone)]
pub struct CalibrationStore {
    path: Option<PathBuf>,
    file: CalibrationFile,
    /// 读取/校验过程中攒下的一句话（文件不存在、解析失败、版本过高……）。
    note: Option<String>,
    /// 文件版本比本程序新 —— **只读**，不许覆盖。
    read_only: bool,
}

impl CalibrationStore {
    /// 不碰磁盘的空表。测试、以及「拿不到配置目录」时用。
    pub fn empty() -> Self {
        CalibrationStore {
            path: None,
            file: CalibrationFile::default(),
            note: None,
            read_only: false,
        }
    }

    /// 从默认位置读取（`%APPDATA%\scope-gui\calib.json`）。
    pub fn load_default() -> Self {
        Self::load_from(persist::default_user_path(DIR_NAME, FILE_NAME))
    }

    /// 从指定位置读取。**任何失败都降级为空表 + 说明**，不返回错误 ——
    /// 标定文件坏了不该让程序起不来。
    pub fn load_from(path: Option<PathBuf>) -> Self {
        let (mut file, note) = persist::load_json::<CalibrationFile>(path.as_deref());
        let mut dropped: Vec<String> = Vec::new();

        // 逐条校验。**单条非法不该毁掉整个文件** —— 丢掉那条并说明，
        // 其余照常可用。serde 拦不住 `volts_per_lsb: -1.0` 这种。
        file.devices.retain(|uid, rec| {
            let before = rec.channels.len();
            rec.channels.retain(|c| {
                c.volts_per_lsb.is_finite() && c.volts_per_lsb > 0.0 && c.zero_lsb.is_finite()
            });
            if rec.channels.len() != before {
                dropped.push(format!("{uid}（{} 个通道）", before - rec.channels.len()));
            }
            !rec.channels.is_empty()
        });

        let read_only = file.version > CALIB_VERSION;
        let mut notes: Vec<String> = Vec::new();
        if let Some(n) = note {
            notes.push(n);
        }
        if !dropped.is_empty() {
            notes.push(format!(
                "标定文件里有 {} 条记录的通道参数非法（volts_per_lsb 必须为有限正数），已丢弃：{}",
                dropped.len(),
                dropped.join("、")
            ));
        }
        if read_only {
            notes.push(format!(
                "标定文件版本为 {}，高于本程序支持的 {}。**只读，不会覆盖它**；\
                 请升级上位机后再修改标定。",
                file.version, CALIB_VERSION
            ));
        } else if file.version < CALIB_VERSION {
            // 低版本：内存里升上来，下次保存即写成新版本。
            file.version = CALIB_VERSION;
        }

        CalibrationStore {
            path,
            file,
            note: if notes.is_empty() {
                None
            } else {
                Some(notes.join(" / "))
            },
            read_only,
        }
    }

    /// 重新读一遍当前路径。连上设备时调用 —— 用户在两次采集之间可能
    /// 在另一个终端跑了 `cal set`。
    ///
    /// **没有路径时什么也不做。** `reload` 的语义是「重新读一遍我来自的那个
    /// 文件」，**不是**「清空」—— 没有文件可读还把手上的内容丢掉，既有破坏性
    /// 又反直觉。注入式用法（测试、将来可能的 `--calib`）会直接踩到：
    /// `connect()` 会调它，于是刚注入的记录在连接的一瞬间被抹掉。
    /// （这条是 MCP 侧的测试逼出来的，不是推出来的。）
    pub fn reload(&mut self) {
        if self.path.is_none() {
            return;
        }
        let path = self.path.clone();
        *self = Self::load_from(path);
    }

    /// 有没有该 uid 的记录。
    pub fn get(&self, uid: &[u8; 12]) -> Option<&DeviceCalib> {
        self.file.devices.get(&encode_uid(uid))
    }

    /// 给一次采集拼出逐通道换算表。**所有调用方都走这里**，不要自己拼。
    pub fn scales_for(&self, uid: Option<&[u8; 12]>, ch_count: usize) -> ScaleSet {
        let uid = uid.copied();
        let Some(rec) = uid.as_ref().and_then(|u| self.get(u)) else {
            return ScaleSet {
                uid,
                ..ScaleSet::uncalibrated(ch_count)
            };
        };
        let mut scales = Vec::with_capacity(ch_count);
        let mut calibrated = Vec::with_capacity(ch_count);
        for ch in 0..ch_count {
            match rec.channels.get(ch) {
                Some(c) => {
                    scales.push(*c);
                    calibrated.push(true);
                }
                None => {
                    scales.push(ChannelScale::default());
                    calibrated.push(false);
                }
            }
        }
        ScaleSet {
            uid,
            scales,
            calibrated,
            record_found: true,
        }
    }

    /// 写入一条通道标定。
    ///
    /// `now_unix` 由调用方给 —— core 不引时间库。
    /// 版本过高时拒绝写入（`Err`），**不静默丢数据**。
    pub fn set_channel(
        &mut self,
        uid: [u8; 12],
        ch: usize,
        scale: ChannelScale,
        note: Option<String>,
        now_unix: u64,
    ) -> Result<(), String> {
        if self.read_only {
            return Err(format!(
                "标定文件版本高于本程序支持的 {CALIB_VERSION}，拒绝写入以免覆盖新字段。\
                 请升级上位机后再改标定。"
            ));
        }
        if !scale.volts_per_lsb.is_finite() || scale.volts_per_lsb <= 0.0 {
            return Err(format!(
                "volts_per_lsb 必须是有限正数（收到 {}）。\
                 若不知该填什么，先量一个已知电压：volts_per_lsb = 参考电压 / (读数 - 零点)",
                scale.volts_per_lsb
            ));
        }
        if !scale.zero_lsb.is_finite() {
            return Err(format!("zero_lsb 必须是有限数（收到 {}）", scale.zero_lsb));
        }

        let rec = self.file.devices.entry(encode_uid(&uid)).or_default();
        if rec.channels.len() <= ch {
            rec.channels.resize(ch + 1, ChannelScale::default());
        }
        rec.channels[ch] = scale;
        if note.is_some() {
            rec.note = note;
        }
        rec.updated_unix = Some(now_unix);
        Ok(())
    }

    /// 写回磁盘。没有路径（拿不到配置目录）时报错 —— **不静默丢弃**。
    pub fn save(&self) -> Result<(), String> {
        if self.read_only {
            return Err("标定文件版本高于本程序，拒绝覆盖".into());
        }
        let p = self.path.as_ref().ok_or_else(|| {
            "没有可用的配置目录（APPDATA / XDG_CONFIG_HOME / HOME 均未设置），无法保存标定"
                .to_string()
        })?;
        persist::save_json_atomic(p, &self.file, WRITE_SUBJECT)
    }

    /// 删除某条记录。用于「这块板子的标定作废了」。
    pub fn remove(&mut self, uid: &[u8; 12]) -> bool {
        self.file.devices.remove(&encode_uid(uid)).is_some()
    }

    /// 读/校验时攒下的说明（文件不存在、解析失败、版本过高、丢弃了非法记录）。
    pub fn note(&self) -> Option<&str> {
        self.note.as_deref()
    }

    /// 文件路径。`cal path` 与 GUI 悬浮提示都用它。
    pub fn path(&self) -> Option<&Path> {
        self.path.as_deref()
    }

    /// 文件版本高于本程序 —— 只读，不许覆盖。
    pub fn is_read_only(&self) -> bool {
        self.read_only
    }

    /// 遍历所有记录（`cal show` 用）。按 uid 排序。
    pub fn iter(&self) -> impl Iterator<Item = (&str, &DeviceCalib)> {
        self.file.devices.iter().map(|(k, v)| (k.as_str(), v))
    }

    /// 记录条数。
    pub fn len(&self) -> usize {
        self.file.devices.len()
    }

    /// 表是不是空的。
    pub fn is_empty(&self) -> bool {
        self.file.devices.is_empty()
    }
}

/// uid → 规范十六进制形式：小写、无分隔、24 字符。
pub fn encode_uid(uid: &[u8; 12]) -> String {
    let mut s = String::with_capacity(24);
    for b in uid {
        s.push_str(&format!("{b:02x}"));
    }
    s
}

/// 十六进制 → uid。**宽容解析**：接受大小写、`:`/`-`/空格分隔、可选的 `0x` 前缀。
///
/// 宽容是有意的：手工敲 24 个十六进制字符很容易顺手加分隔符。
/// 写回去时一律走 [`encode_uid`]，所以规范形式只有一种。
pub fn decode_uid(s: &str) -> Option<[u8; 12]> {
    let cleaned: String = s
        .trim()
        .trim_start_matches("0x")
        .trim_start_matches("0X")
        .chars()
        .filter(|c| !matches!(c, ':' | '-' | ' ' | '_'))
        .collect();
    if cleaned.len() != 24 {
        return None;
    }
    let mut uid = [0u8; 12];
    for i in 0..12 {
        uid[i] = u8::from_str_radix(&cleaned[i * 2..i * 2 + 2], 16).ok()?;
    }
    Some(uid)
}

/// 已知的**占位** uid —— 拿它标定等于给「所有共用这个占位值的板子」标定。
///
/// 两个来源：
/// - 模拟器（`sim/src/device.rs`）：每个 `SimDevice` 都报同一个 uid
/// - 固件（`firmware/App/proto_task.c`）：真 uid 要从 MCU 的 `0x1FFFF7E8`
///   读，那属于 `Hardware/`，还没写
///
/// 返回一句说明，不是占位值就返回 `None`。
pub fn is_placeholder_uid(uid: &[u8; 12]) -> Option<&'static str> {
    const SIM: [u8; 12] = [
        0x5E, 0xED, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
    ];
    const FIRMWARE: [u8; 12] = [
        0xAA, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
    ];
    if *uid == SIM {
        return Some(
            "这是**模拟器**的固定 uid —— 每个模拟器实例都报同一串，\
             标定它会作用于之后所有模拟器会话。",
        );
    }
    if *uid == FIRMWARE {
        return Some(
            "这是**固件的占位 uid**（真 uid 应当从 MCU 的 0x1FFFF7E8 读，尚未实现）。\
             标定它会作用于所有仍跑占位固件的板子；请务必在 note 里写清是哪块板。",
        );
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    fn uid_of(s: &str) -> [u8; 12] {
        decode_uid(s).expect("测试用的 uid 串应当合法")
    }

    fn scale(v: f64, z: f64) -> ChannelScale {
        ChannelScale {
            volts_per_lsb: v,
            zero_lsb: z,
        }
    }

    fn temp_store(name: &str) -> CalibrationStore {
        let dir = std::env::temp_dir().join(format!("scope-calib-{name}"));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        CalibrationStore::load_from(Some(dir.join("calib.json")))
    }

    #[test]
    fn uid_round_trips_through_the_canonical_form() {
        let uid = uid_of("5EED00010203040506070809");
        assert_eq!(encode_uid(&uid), "5eed00010203040506070809");
        assert_eq!(decode_uid(&encode_uid(&uid)).unwrap(), uid);
    }

    /// 宽容解析：手工敲 24 个十六进制字符时顺手加分隔符是常事，
    /// 而**两种拼法会静默产生两条记录** —— 那是补不回来的迁移 bug。
    #[test]
    fn uid_parsing_tolerates_separators_and_case() {
        let want = uid_of("5eed00010203040506070809");
        for s in [
            "5E:ED:00:01:02:03:04:05:06:07:08:09",
            "5e-ed-00-01-02-03-04-05-06-07-08-09",
            "0x5eed00010203040506070809",
            "  5EED00010203040506070809  ",
        ] {
            assert_eq!(decode_uid(s), Some(want), "应能解析 {s}");
        }
        assert_eq!(decode_uid("5eed"), None, "长度不对必须拒绝");
        assert_eq!(
            decode_uid("5eed0001020304050607080g"),
            None,
            "非十六进制必须拒绝"
        );
    }

    #[test]
    fn an_empty_store_yields_placeholder_scales() {
        let s = CalibrationStore::empty();
        let set = s.scales_for(Some(&uid_of("5eed00010203040506070809")), 2);
        assert!(!set.record_found());
        assert!(!set.any_calibrated());
        assert!(!set.all_calibrated());
        assert_eq!(set.as_slice().len(), 2, "长度必须等于通道数");
        assert_eq!(
            set.get(0).volts_per_lsb,
            ChannelScale::default().volts_per_lsb
        );
        assert_eq!(
            set.get(99).volts_per_lsb,
            ChannelScale::default().volts_per_lsb
        );
        assert!(set.summary_note().contains("未标定"));
    }

    /// `all_calibrated()` 在空集上必须是 `false` —— 「一个通道都没有」
    /// 不该被读成「全都标定好了」。
    #[test]
    fn all_calibrated_is_false_for_an_empty_channel_set() {
        assert!(!ScaleSet::uncalibrated(0).all_calibrated());
        assert!(!ScaleSet::uncalibrated(1).all_calibrated());
    }

    #[test]
    fn a_record_round_trips_through_the_file() {
        let mut s = temp_store("roundtrip");
        let uid = uid_of("5eed00010203040506070809");
        s.set_channel(
            uid,
            0,
            scale(1.0e-3, 2048.0),
            Some("台架 1.000 V".into()),
            1_780_560_000,
        )
        .unwrap();
        s.save().unwrap();

        let again = CalibrationStore::load_from(s.path().map(Path::to_path_buf));
        let set = again.scales_for(Some(&uid), 1);
        assert!(set.record_found());
        assert!(set.all_calibrated());
        assert_eq!(set.get(0).volts_per_lsb, 1.0e-3);
        assert!(set.summary_note().contains("5eed00010203040506070809"));
        assert_eq!(
            again.get(&uid).unwrap().note.as_deref(),
            Some("台架 1.000 V")
        );
    }

    /// 记录里只有一个通道、设备报两个通道 —— 缺的那个退回占位值，
    /// 且必须**如实报告为未标定**，不能因为「有记录」就说整个设备标定好了。
    #[test]
    fn a_missing_channel_is_reported_as_uncalibrated() {
        let mut s = temp_store("partial");
        let uid = uid_of("5eed00010203040506070809");
        s.set_channel(uid, 0, scale(1.0e-3, 2048.0), None, 0)
            .unwrap();

        let set = s.scales_for(Some(&uid), 2);
        assert!(set.record_found(), "记录是有的");
        assert!(set.is_calibrated(0));
        assert!(!set.is_calibrated(1), "通道 1 没有记录");
        assert!(set.any_calibrated());
        assert!(!set.all_calibrated());
        assert_eq!(set.as_slice().len(), 2);
        assert!(set.summary_note().contains("CH2"), "应点名缺哪个通道");
    }

    /// 单条记录非法时**丢掉那一条**，其余照常可用 —— 并说明丢了什么。
    /// serde 拦不住 `volts_per_lsb: -1.0`。
    #[test]
    fn invalid_channels_are_dropped_with_a_note_not_fatal() {
        let dir = std::env::temp_dir().join("scope-calib-invalid");
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        let p = dir.join("calib.json");
        std::fs::write(
            &p,
            r#"{"version":1,"devices":{
                "5eed00010203040506070809":{"channels":[{"volts_per_lsb":-1.0,"zero_lsb":0.0}]},
                "aabbccddeeff001122334455":{"channels":[{"volts_per_lsb":0.001,"zero_lsb":2048.0}]}
            }}"#,
        )
        .unwrap();

        let s = CalibrationStore::load_from(Some(p));
        let bad = uid_of("5eed00010203040506070809");
        let good = uid_of("aabbccddeeff001122334455");
        assert!(s.get(&bad).is_none(), "非法记录必须被丢掉");
        assert!(s.get(&good).is_some(), "合法记录必须留下");
        assert!(s.note().unwrap().contains("非法"), "必须说明丢了什么");
    }

    /// 版本高于本程序 → **只读**，拒绝写入，且说明原因。
    /// 这条是「旧程序覆盖新程序写的字段」的唯一防线。
    #[test]
    fn a_newer_file_version_is_read_only() {
        let dir = std::env::temp_dir().join("scope-calib-newver");
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        let p = dir.join("calib.json");
        std::fs::write(
            &p,
            r#"{"version":99,"devices":{"aabbccddeeff001122334455":{"channels":[{"volts_per_lsb":0.001,"zero_lsb":2048.0}]}}}"#,
        )
        .unwrap();

        let mut s = CalibrationStore::load_from(Some(p));
        assert!(s.is_read_only());
        assert!(
            s.get(&uid_of("aabbccddeeff001122334455")).is_some(),
            "仍要能读"
        );
        let e = s
            .set_channel(
                uid_of("aabbccddeeff001122334455"),
                0,
                scale(1.0e-3, 0.0),
                None,
                0,
            )
            .unwrap_err();
        assert!(e.contains("拒绝写入"), "实测：{e}");
        assert!(s.save().is_err(), "只读时必须拒绝保存");
    }

    #[test]
    fn non_positive_or_non_finite_scales_are_refused() {
        let mut s = temp_store("badscale");
        let uid = uid_of("5eed00010203040506070809");
        for bad in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            let e = s
                .set_channel(uid, 0, scale(bad, 2048.0), None, 0)
                .expect_err("非法换算必须被拒");
            assert!(e.contains("volts_per_lsb"), "实测：{e}");
        }
        let e = s
            .set_channel(uid, 0, scale(1.0e-3, f64::NAN), None, 0)
            .expect_err("非法零点必须被拒");
        assert!(e.contains("zero_lsb"), "实测：{e}");
    }

    /// `reload()` 在**没有路径**时不得清空内容。
    ///
    /// 回归：`connect()` 每次都调 `reload()`，而它原先是无条件
    /// `*self = load_from(path)` —— 于是注入进来的记录在连接的一瞬间被抹掉，
    /// MCP 侧那条「注入标定后数值应缩放」的测试直接失败。
    #[test]
    fn reload_without_a_path_keeps_the_contents() {
        let mut s = CalibrationStore::empty();
        let uid = uid_of("5eed00010203040506070809");
        s.set_channel(uid, 0, scale(1.0e-3, 2048.0), None, 0)
            .unwrap();
        assert!(s.get(&uid).is_some());

        s.reload();
        assert!(s.get(&uid).is_some(), "没有路径可读时不该把手上的记录丢掉");
    }

    /// 有路径时 `reload()` 必须**真的重新读盘** ——
    /// 用户在两次采集之间跑了 `cal set`，界面得跟上。
    #[test]
    fn reload_with_a_path_picks_up_external_changes() {
        let mut s = temp_store("reload");
        let uid = uid_of("5eed00010203040506070809");
        s.set_channel(uid, 0, scale(1.0e-3, 2048.0), None, 0)
            .unwrap();
        s.save().unwrap();
        assert!(s.get(&uid).is_some());

        // 模拟「另一个终端把文件改没了」
        let p = s.path().unwrap().to_path_buf();
        std::fs::write(&p, r#"{"version":1,"devices":{}}"#).unwrap();

        s.reload();
        assert!(s.get(&uid).is_none(), "有路径时必须重新读盘");
    }

    /// 占位 uid 认得出，真 uid 不误报。
    #[test]
    fn placeholder_uids_are_recognised() {
        assert!(is_placeholder_uid(&uid_of("5eed00010203040506070809")).is_some());
        assert!(is_placeholder_uid(&uid_of("aa5500000000000000000001")).is_some());
        assert!(is_placeholder_uid(&uid_of("0123456789abcdef01234567")).is_none());
    }

    /// 写通道 1 时不能把通道 0 顶掉 —— `resize` 的填充值必须是占位，
    /// 且已有的值要原样保留。
    #[test]
    fn writing_a_high_channel_keeps_the_lower_one() {
        let mut s = temp_store("sparse");
        let uid = uid_of("5eed00010203040506070809");
        s.set_channel(uid, 0, scale(1.0e-3, 2048.0), None, 0)
            .unwrap();
        s.set_channel(uid, 2, scale(2.0e-3, 1000.0), None, 0)
            .unwrap();

        let rec = s.get(&uid).unwrap();
        assert_eq!(rec.channels.len(), 3);
        assert_eq!(rec.channels[0].volts_per_lsb, 1.0e-3, "通道 0 被顶掉了");
        assert_eq!(rec.channels[2].volts_per_lsb, 2.0e-3);
        // 中间那个没写过，是占位值（不等于前后两者）
        assert_eq!(
            rec.channels[1].volts_per_lsb,
            ChannelScale::default().volts_per_lsb
        );
    }
}
