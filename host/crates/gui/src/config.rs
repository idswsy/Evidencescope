//! AI 面板的配置 —— API key / 端点 / 模型。
//!
//! # 这个文件为什么写得这么小心
//!
//! 用户选的是**把 key 存在本地配置文件里**。这意味着两件事必须做到：
//!
//! 1. **密钥不得落入项目目录**。这是唯一会造成实质损害的失败模式，
//!    且已有先例。
//! 2. **不能谎称它安全**。Windows 下 `std` 设置不了 ACL，所以它就是**明文**。
//!    文档和界面都这么说，不写「已加密」这种话。
//!
//! # 纵深防护（四道，任何一道单独都不够）
//!
//! 1. **路径解析绝不回退**。两个环境变量都没有 → 返回 `None`，配置只留在内存里。
//!    这是最要紧的一道：回退到「当前目录」或「exe 旁边」正是 key 写入仓库的
//!    唯一真实入口，因此不提供该回退路径。
//! 2. **写之前断言目标不在仓库里**。[`check_writable`] 会把双方都
//!    canonicalize 再比前缀 —— 光是字符串比较挡不住 `..` 和符号链接。
//! 3. **Unix 下 `0600`**。Windows 下照实说做不到。
//! 4. **原子写**（同目录临时文件 + rename）。写入中断留下的半截文件，
//!    比文件不存在更难诊断。
//!
//! 路径解析写成**可注入 env 的纯函数**，所以 CI 能在无头机上把所有分支测完。
//!
//! # 这套机械现在在 `scope_core::persist`
//!
//! 上面四道防线原本实现在本文件里。标定表（`scope_core::calib`）要复用同一套，
//! 所以**上移到 core**，本文件只留 [`AiConfig`] 与几个**同签名的薄包装** ——
//! 闸门那类代码不该有两份：两份就会各自腐烂，而且总有一份先烂。
//!
//! 本文件原有的测试**一条都没改**。它们现在是双层证据：
//! 既是 AI 配置本身的回归，也是「搬运没有改变行为」的回归。

use std::path::{Path, PathBuf};

/// 配置文件名。
const FILE_NAME: &str = "config.json";
/// 配置目录名（在 `%APPDATA%` / `$XDG_CONFIG_HOME` 之下）。
const DIR_NAME: &str = "scope-gui";

/// 默认端点 —— DeepSeek 的 Anthropic 兼容地址。
///
/// 与 `tools/agent_demo/agent.py` 的 `AGENT_BASE_URL` 默认值一致：
/// 那边的接口已验证可用，形状（`x-api-key` + `anthropic-version`）是验证过的。
pub const DEFAULT_BASE_URL: &str = "https://api.deepseek.com/anthropic/v1/messages";

/// 默认模型。
pub const DEFAULT_MODEL: &str = "deepseek-flash";

/// AI 面板的配置。
#[derive(Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct AiConfig {
    /// API key。**明文** —— 见模块头。
    pub api_key: String,
    /// 端点（完整的 messages URL）。
    pub base_url: String,
    /// 模型名。
    pub model: String,
}

impl Default for AiConfig {
    fn default() -> Self {
        AiConfig {
            api_key: String::new(),
            base_url: DEFAULT_BASE_URL.to_string(),
            model: DEFAULT_MODEL.to_string(),
        }
    }
}

impl AiConfig {
    /// key 填了没有（空串和纯空白都算没填）。
    pub fn has_key(&self) -> bool {
        !self.api_key.trim().is_empty()
    }

    /// 端点是 http:// 还是 https:// —— 界面要拿它提醒「key 会明文上网」。
    pub fn is_plaintext_endpoint(&self) -> bool {
        self.base_url.trim_start().starts_with("http://")
    }
}

/// **手写的 `Debug`，故意不 derive** —— derive 会把 key 原样打出来。
///
/// 这条不是洁癖：`{:?}` 是日志、`unwrap()` 的 panic 消息、断言失败信息里
/// 默认会用到的东西。derive 一写上去，key 就会在某个没人注意的地方进日志。
///
/// 所以这里显式遮蔽 key，而且**测试钉住了这一点**。
impl std::fmt::Debug for AiConfig {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("AiConfig")
            .field("api_key", &redact(&self.api_key))
            .field("base_url", &self.base_url)
            .field("model", &self.model)
            .finish()
    }
}

/// 把 key 遮成 `sk-1***abcd` 这种可辨认但不可用的形式。
///
/// 保留首尾各几个字符是有用的：用户能看出「我用的是哪个 key」，
/// 但看不出完整内容。太短就整个遮掉，免得遮了等于没遮。
pub fn redact(key: &str) -> String {
    let k = key.trim();
    if k.is_empty() {
        return "(未设置)".to_string();
    }
    let n = k.chars().count();
    if n <= 8 {
        return "*".repeat(n);
    }
    let head: String = k.chars().take(4).collect();
    let tail: String = k.chars().skip(n - 4).collect();
    format!("{head}…{tail}（{n} 字符）")
}

/// 解析配置文件路径。**两个环境变量都没有就返回 `None`，不回退。**
///
/// `env` 是注入的，所以 CI 能把「Windows 有 APPDATA」「Unix 有 XDG_CONFIG_HOME」
/// 「Unix 只有 HOME」「全都没有」四条分支都测一遍。
///
/// 顺序：
/// - `APPDATA`（Windows）→ `%APPDATA%\scope-gui\config.json`
/// - `XDG_CONFIG_HOME`（Unix）→ `$XDG_CONFIG_HOME/scope-gui/config.json`
/// - `HOME`（Unix 兜底）→ `$HOME/.config/scope-gui/config.json`
/// - 都没有 → `None`
///
/// **故意不看当前目录，也不看 exe 所在目录。**
/// **只在测试里用。** 生产路径走 [`default_config_path`]（直接调 core）。
///
/// 留着这个包装是因为本文件原有的四条路径解析测试都调它 ——
/// 改测试去调 core 就把「搬运没有改变行为」那层证据弄丢了。
#[cfg(test)]
pub fn resolve_config_path(env: &dyn Fn(&str) -> Option<String>) -> Option<PathBuf> {
    scope_core::persist::resolve_user_path(env, DIR_NAME, FILE_NAME)
}

/// 用真实环境变量解析路径。
pub fn default_config_path() -> Option<PathBuf> {
    scope_core::persist::default_user_path(DIR_NAME, FILE_NAME)
}

/// 读取配置。**任何失败均降级为默认值**，不返回错误。
///
/// 理由：配置文件损坏不应导致应用无法启动。密钥无效时，用户会在首次分析时
/// 得到「密钥被拒绝」及相应处理措施；而「配置文件第 7 行解析失败」这一类信息
/// 对用户不具可操作性。
///
/// 返回 `(配置, 来源说明)`。来源说明描述读取结果（文件不存在 / 解析失败 /
/// 无可用目录），供调用方记入日志。
pub fn load(path: Option<&Path>) -> (AiConfig, Option<String>) {
    scope_core::persist::load_json::<AiConfig>(path)
}

/// 写入失败时，用它点名「写的是什么」。
///
/// 提成常量而不是两处各写一个字面量：分别写的话，`save` 那处漂移了
/// **没有任何测试会红** —— 对抗性核查实测把 `save` 的字面量改成「标定数据」，
/// 75 条 gui 测试全过。原因见 [`check_writable`] 的说明。
/// 值本身由 `the_refusal_message_names_what_was_being_written` 钉住。
const WRITE_SUBJECT: &str = "API 密钥";

/// 保存配置。失败时返回的是**带「怎么办」的中文说明**，不是一个裸错误码。
pub fn save(path: &Path, cfg: &AiConfig) -> Result<(), String> {
    scope_core::persist::save_json_atomic(path, cfg, WRITE_SUBJECT)
}

/// 保存前的安全闸门 —— 实现在 [`scope_core::persist::check_writable`]。
///
/// **只在测试里用**：生产路径走 [`save`]，它直接调 core。
/// 保留**两参数**的旧签名，为的是让本文件原有的三条闸门测试一个字都不用改 ——
/// 它们同时是「搬运没有改变行为」的证据。
/// 新代码请直接用 core 那个带 `what` 的版本。
#[cfg(test)]
pub fn check_writable(parent: &Path, forbidden_root: &Path) -> Result<(), String> {
    scope_core::persist::check_writable(parent, forbidden_root, WRITE_SUBJECT)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 造一个假的 env。持有所有权，这样返回的闭包是 `'static` 的，
    /// 不会被调用处的临时数组生命周期绊住。
    fn env_of(pairs: &[(&str, &str)]) -> impl Fn(&str) -> Option<String> + 'static {
        let owned: Vec<(String, String)> = pairs
            .iter()
            .map(|(k, v)| ((*k).to_string(), (*v).to_string()))
            .collect();
        move |k| {
            owned
                .iter()
                .find(|(name, _)| name == k)
                .map(|(_, v)| v.clone())
        }
    }

    // ── 路径解析：四条分支，一条都不能漏 ─────────────────────────────

    #[test]
    fn appdata_wins_on_windows() {
        let env = env_of(&[
            ("APPDATA", r"C:\Users\me\AppData\Roaming"),
            ("HOME", "/home/me"),
        ]);
        let p = resolve_config_path(&env).expect("应当解析出来");
        assert!(p.ends_with("scope-gui/config.json") || p.ends_with(r"scope-gui\config.json"));
        assert!(
            p.to_string_lossy().contains("AppData"),
            "APPDATA 应当优先：{p:?}"
        );
    }

    #[test]
    fn xdg_beats_home_on_unix() {
        let env = env_of(&[("XDG_CONFIG_HOME", "/xdg"), ("HOME", "/home/me")]);
        let p = resolve_config_path(&env).unwrap();
        assert_eq!(p, Path::new("/xdg").join(DIR_NAME).join(FILE_NAME));
    }

    #[test]
    fn home_is_the_last_resort() {
        let env = env_of(&[("HOME", "/home/me")]);
        let p = resolve_config_path(&env).unwrap();
        assert_eq!(
            p,
            Path::new("/home/me")
                .join(".config")
                .join(DIR_NAME)
                .join(FILE_NAME)
        );
    }

    /// **最要紧的一条。** 一个环境变量都没有时必须返回 `None`。
    ///
    /// 如果这里改成「回退到当前目录」，key 就会直接写入仓库 ——
    /// 那是这个功能唯一能造成真实伤害的失败模式。
    #[test]
    fn no_env_means_no_path_never_a_fallback() {
        let env = env_of(&[]);
        assert!(
            resolve_config_path(&env).is_none(),
            "没有配置目录时必须返回 None —— 绝不许回退到当前目录或 exe 旁边"
        );
    }

    #[test]
    fn blank_env_values_are_treated_as_absent() {
        // 空串和纯空白都不算「设了」—— 否则会解析出相对路径 `.`，
        // 那又变相地把 key 放到了当前目录
        let env = env_of(&[("APPDATA", "   "), ("XDG_CONFIG_HOME", "")]);
        assert!(resolve_config_path(&env).is_none());
    }

    // ── 安全闸门 ─────────────────────────────────────────────────────

    #[test]
    fn refuses_to_write_inside_the_forbidden_root() {
        let root = std::env::temp_dir().join("scope-gui-test-repo");
        let inside = root.join("host").join("crates");
        std::fs::create_dir_all(&inside).unwrap();

        let err = check_writable(&inside, &root).expect_err("在项目目录内必须拒写");
        assert!(err.contains("拒绝写入"), "错误须说明是拒写：{err}");
        assert!(err.contains("不得落入项目目录"), "错误须说明原因：{err}");

        let _ = std::fs::remove_dir_all(&root);
    }

    /// 拒写消息必须点名**写的是什么** —— 那是唯一会告诉用户「放错了什么」的地方。
    ///
    /// 这条**必须走生产的 [`save`]**，不能走上面那个 `check_writable`：
    /// 那个是 `#[cfg(test)]` 包装，它自己也有一个 `what` 参数，
    /// 所以它钉不住 `save` 实际传了什么。
    ///
    /// 对抗性核查实测：把 `save` 里的 `WRITE_SUBJECT` 改成 `"标定数据"`，
    /// **75 条 gui 测试全过**，而用户看到的拒写消息已经错了。
    /// 现在两处共用一个常量，且这条测试钉住它的值。
    #[test]
    fn the_refusal_message_names_what_was_being_written() {
        let cwd = std::env::current_dir().expect("测试要能拿到当前目录");
        let target = cwd.join("scope-gui-must-never-be-written.json");

        let err = save(&target, &AiConfig::default()).expect_err("仓库目录内必须拒写");
        assert!(
            err.contains("API 密钥不得落入项目目录"),
            "拒写消息必须点名写的是什么，实测：{err}"
        );
        assert!(!target.exists(), "闸门必须在落盘之前拦住");
    }

    #[test]
    fn allows_writing_outside_the_forbidden_root() {
        let root = std::env::temp_dir().join("scope-gui-test-root");
        let safe = std::env::temp_dir().join("scope-gui-test-safe");
        std::fs::create_dir_all(&root).unwrap();
        std::fs::create_dir_all(&safe).unwrap();

        check_writable(&safe, &root).expect("仓库之外应当允许");

        let _ = std::fs::remove_dir_all(&root);
        let _ = std::fs::remove_dir_all(&safe);
    }

    /// `..` 必须被识破 —— 纯字符串前缀比较挡不住它。
    #[test]
    fn dotdot_cannot_sneak_past_the_gate() {
        let root = std::env::temp_dir().join("scope-gui-test-dd");
        let sub = root.join("sub");
        std::fs::create_dir_all(&sub).unwrap();

        // sub/../sub 字面上不以 root 开头，但真实路径就在 root 里面
        let sneaky = sub.join("..").join("sub");
        let err = check_writable(&sneaky, &root).expect_err("`..` 绕不过去");
        assert!(err.contains("拒绝写入"));

        let _ = std::fs::remove_dir_all(&root);
    }

    // ── key 脱敏 ─────────────────────────────────────────────────────

    /// `Debug` 必须遮蔽 key。**这条一旦失守，key 就会进日志。**
    #[test]
    fn debug_output_never_contains_the_raw_key() {
        let secret = "sk-1234567890abcdefSECRET";
        let cfg = AiConfig {
            api_key: secret.to_string(),
            ..Default::default()
        };
        let printed = format!("{cfg:?}");
        assert!(
            !printed.contains(secret),
            "Debug 里出现了完整 key —— 它迟早会进日志：{printed}"
        );
        assert!(
            !printed.contains("abcdefSECRET"),
            "Debug 里出现了 key 的尾部：{printed}"
        );
        assert!(
            printed.contains("16 字符") || printed.contains("字符"),
            "应当给出长度：{printed}"
        );
    }

    #[test]
    fn redact_handles_short_and_empty_keys() {
        assert_eq!(redact(""), "(未设置)");
        assert_eq!(redact("   "), "(未设置)");
        // 太短的整个遮掉 —— 遮了等于没遮不如不遮
        assert_eq!(redact("sk-123"), "******");
        assert!(!redact("sk-abcdefghijkl").contains("efghijkl"));
    }

    /// 脱敏后的串本身不能是可用的 key（首尾各 4 字符 + 省略号，拼不回去）。
    #[test]
    fn redacted_form_is_not_a_usable_key() {
        let key = "sk-aaaaBBBBccccDDDD";
        let r = redact(key);
        assert_ne!(r, key);
        assert!(!r.contains("BBBB"), "中间的字符不能露出来");
    }

    // ── 端点 ─────────────────────────────────────────────────────────

    #[test]
    fn plaintext_endpoint_is_detected() {
        let c = AiConfig {
            base_url: "http://evil.example/v1/messages".into(),
            ..Default::default()
        };
        assert!(c.is_plaintext_endpoint(), "http:// 必须被认出来");
        assert!(
            !AiConfig::default().is_plaintext_endpoint(),
            "默认端点是 https"
        );
    }

    #[test]
    fn has_key_ignores_whitespace() {
        assert!(!AiConfig::default().has_key());
        let blank = AiConfig {
            api_key: "  \n ".into(),
            ..Default::default()
        };
        assert!(!blank.has_key(), "纯空白不算填了 key");
    }

    // ── 往返 ─────────────────────────────────────────────────────────

    #[test]
    fn config_survives_a_json_round_trip() {
        let cfg = AiConfig {
            api_key: "sk-test".into(),
            base_url: "https://example.test/v1/messages".into(),
            model: "some-model".into(),
        };
        let text = serde_json::to_string(&cfg).unwrap();
        let back: AiConfig = serde_json::from_str(&text).unwrap();
        assert_eq!(cfg, back);
    }

    /// 缺字段的旧配置要能读 —— `#[serde(default)]` 管这个。
    #[test]
    fn partial_json_fills_in_defaults() {
        let cfg: AiConfig = serde_json::from_str(r#"{"api_key":"sk-x"}"#).unwrap();
        assert_eq!(cfg.api_key, "sk-x");
        assert_eq!(cfg.base_url, DEFAULT_BASE_URL);
        assert_eq!(cfg.model, DEFAULT_MODEL);
    }

    #[test]
    fn corrupt_config_degrades_instead_of_panicking() {
        let dir = std::env::temp_dir().join("scope-gui-test-corrupt");
        std::fs::create_dir_all(&dir).unwrap();
        let p = dir.join(FILE_NAME);
        std::fs::write(&p, "{ this is not json").unwrap();

        let (cfg, note) = load(Some(&p));
        assert_eq!(cfg, AiConfig::default(), "坏文件应降级成默认值");
        assert!(note.is_some(), "要说清为什么用了默认值");
        assert!(note.unwrap().contains("无法解析"), "说明里要讲清是解析失败");

        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn missing_file_is_not_an_error() {
        let p = std::env::temp_dir()
            .join("scope-gui-test-missing")
            .join(FILE_NAME);
        let (cfg, note) = load(Some(&p));
        assert_eq!(cfg, AiConfig::default());
        assert!(note.unwrap().contains("尚不存在"));
    }

    #[test]
    fn no_path_at_all_says_so() {
        let (_, note) = load(None);
        assert!(
            note.unwrap().contains("仅保存在内存中"),
            "无配置目录时须说明设置不会被持久化"
        );
    }
}
