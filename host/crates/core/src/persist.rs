//! 用户目录下的 JSON 状态文件：路径解析、降级读取、原子写、防泄漏闸门。
//!
//! 这套机械原本在 `scope-gui::config` 里，为 AI 面板的密钥而写。标定表
//! （[`crate::calib`]）要复用同一套，所以上移到 core —— **防泄漏闸门这类东西
//! 不该有两份**：两份就会各自腐烂，而且总有一份先烂。
//!
//! # 纵深防护（四道，任何一道单独都不够）
//!
//! 1. **路径解析绝不回退**。两个环境变量都没有 → 返回 `None`，状态只留在内存里。
//!    这是最要紧的一道：回退到「当前目录」或「exe 旁边」正是文件写入仓库的
//!    唯一真实入口，因此不提供该回退路径。
//! 2. **写之前断言目标不在仓库里**。[`check_writable`] 会把双方都
//!    canonicalize 再比前缀 —— 光是字符串比较挡不住 `..` 和符号链接。
//! 3. **Unix 下 `0600`**。Windows 下照实说做不到 —— 见 [`write_private`]。
//! 4. **原子写**（同目录临时文件 + rename）。写入中断留下的半截文件，
//!    比文件不存在更难诊断。
//!
//! 路径解析写成**可注入 env 的纯函数**，所以 CI 能在无头机上把所有分支测完。
//!
//! # `what` 这个参数
//!
//! [`save_json_atomic`] 与 [`check_writable`] 收一个 `what: &str`，是「写的是什么」
//! （如 `"API 密钥"`、`"标定数据"`），只影响措辞。但闸门拒绝时**它必须出现**——
//! 那条消息是唯一会告诉用户「你把什么东西放错了地方」的地方。

use std::path::{Path, PathBuf};

use serde::de::DeserializeOwned;
use serde::Serialize;

/// 解析用户目录下的状态文件路径。**两个环境变量都没有就返回 `None`，不回退。**
///
/// `env` 是注入的，所以 CI 能把「Windows 有 APPDATA」「Unix 有 XDG_CONFIG_HOME」
/// 「Unix 只有 HOME」「全都没有」四条分支都测一遍。
///
/// 顺序：
/// - `APPDATA`（Windows）→ `%APPDATA%\<dir>\<file>`
/// - `XDG_CONFIG_HOME`（Unix）→ `$XDG_CONFIG_HOME/<dir>/<file>`
/// - `HOME`（Unix 兜底）→ `$HOME/.config/<dir>/<file>`
/// - 都没有 → `None`
///
/// **故意不看当前目录，也不看 exe 所在目录。**
pub fn resolve_user_path(
    env: &dyn Fn(&str) -> Option<String>,
    dir: &str,
    file: &str,
) -> Option<PathBuf> {
    let non_empty = |k: &str| env(k).filter(|v| !v.trim().is_empty());

    if let Some(appdata) = non_empty("APPDATA") {
        return Some(Path::new(&appdata).join(dir).join(file));
    }
    if let Some(xdg) = non_empty("XDG_CONFIG_HOME") {
        return Some(Path::new(&xdg).join(dir).join(file));
    }
    if let Some(home) = non_empty("HOME") {
        return Some(Path::new(&home).join(".config").join(dir).join(file));
    }
    None
}

/// 用真实环境变量解析路径。
pub fn default_user_path(dir: &str, file: &str) -> Option<PathBuf> {
    resolve_user_path(&|k| std::env::var(k).ok(), dir, file)
}

/// 读取 JSON 状态文件。**任何失败均降级为默认值**，不返回错误。
///
/// 理由：文件损坏不应导致程序无法启动。而「文件第 7 行解析失败」这类信息
/// 对用户不具可操作性。
///
/// 返回 `(值, 来源说明)`。来源说明描述读取结果（文件不存在 / 解析失败 /
/// 无可用目录），供调用方记入日志或界面提示。
pub fn load_json<T>(path: Option<&Path>) -> (T, Option<String>)
where
    T: DeserializeOwned + Default,
{
    let Some(p) = path else {
        return (
            T::default(),
            Some(
                "未找到配置目录（APPDATA / XDG_CONFIG_HOME / HOME 均未设置）。\
                 本次修改仅保存在内存中。"
                    .into(),
            ),
        );
    };
    match std::fs::read_to_string(p) {
        Ok(text) => match serde_json::from_str::<T>(&text) {
            Ok(v) => (v, None),
            Err(e) => (
                T::default(),
                Some(format!(
                    "{} 内容无法解析（{e}）。已改用默认值；执行一次保存即可覆盖",
                    p.display()
                )),
            ),
        },
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => (
            T::default(),
            Some(format!("{} 尚不存在。保存一次后将自动创建", p.display())),
        ),
        Err(e) => (
            T::default(),
            Some(format!(
                "读取 {} 失败：{e}。已改用默认值；请检查文件权限",
                p.display()
            )),
        ),
    }
}

/// 原子地写 JSON 状态文件，写之前先过一遍防泄漏闸门。
///
/// 失败时返回的是**带「怎么办」的中文说明**，不是一个裸错误码。
/// `what` 是「写的是什么」，用于措辞。
pub fn save_json_atomic<T>(path: &Path, value: &T, what: &str) -> Result<(), String>
where
    T: Serialize,
{
    let parent = path
        .parent()
        .ok_or_else(|| format!("{} 无上级目录，无法保存", path.display()))?;

    let cwd = std::env::current_dir()
        .map_err(|e| format!("无法获取当前工作目录（{e}）。无法确认目标位置安全性，拒绝写入"))?;
    check_writable(parent, &cwd, what)?;

    let body = serde_json::to_string_pretty(value)
        .map_err(|e| format!("{what}序列化失败：{e}。属程序缺陷，请提交 issue"))?;

    // 原子写：同目录临时文件 + rename。
    // 同目录是必须的 —— 跨文件系统的 rename 不是原子操作。
    // 临时名取自目标文件名，同一目录下的两个状态文件各有各的临时文件。
    let stem = path
        .file_name()
        .ok_or_else(|| format!("{} 无文件名，无法保存", path.display()))?;
    let mut tmp_name = stem.to_os_string();
    tmp_name.push(".tmp");
    let tmp = parent.join(tmp_name);

    write_private(&tmp, body.as_bytes())
        .map_err(|e| format!("写入 {} 失败：{e}。请检查目录权限", tmp.display()))?;
    std::fs::rename(&tmp, path).map_err(|e| {
        let _ = std::fs::remove_file(&tmp);
        format!(
            "替换 {} 失败：{e}。请确认该文件未被其他程序占用",
            path.display()
        )
    })?;
    Ok(())
}

/// 写文件，Unix 下带 `0600` 权限。
///
/// **Windows 下 `std` 设置不了 ACL** —— 文件会是默认权限（通常同用户可读）。
/// 该机制**不声称**已提供保护。
fn write_private(path: &Path, bytes: &[u8]) -> std::io::Result<()> {
    #[cfg(unix)]
    {
        use std::io::Write as _;
        use std::os::unix::fs::OpenOptionsExt as _;
        let mut f = std::fs::OpenOptions::new()
            .write(true)
            .create(true)
            .truncate(true)
            .mode(0o600)
            .open(path)?;
        f.write_all(bytes)?;
        Ok(())
    }
    #[cfg(not(unix))]
    {
        std::fs::write(path, bytes)
    }
}

/// 保存前的安全闸门：**目标目录不许在当前工作目录（也就是仓库）里面**。
///
/// 两边都 canonicalize 之后再比 —— 纯字符串比较挡不住 `..`、
/// 符号链接、以及 Windows 的短路径名。
///
/// `parent` 不存在时先建出来再判断（因为 canonicalize 要求路径存在）。
/// `what` 只影响措辞，但**必须出现在拒绝消息里** —— 那是唯一会告诉用户
/// 「你把什么东西放错了地方」的地方。
pub fn check_writable(parent: &Path, forbidden_root: &Path, what: &str) -> Result<(), String> {
    std::fs::create_dir_all(parent)
        .map_err(|e| format!("创建目录 {} 失败：{e}。请检查权限", parent.display()))?;

    let real_parent = std::fs::canonicalize(parent)
        .map_err(|e| format!("解析路径 {} 失败：{e}", parent.display()))?;
    // 当前目录拿不到真实路径时**不放弃检查** —— 退回按字面路径比，
    // 允许误报，不得漏报（该闸门用于防止密钥进入仓库）。
    let real_root = std::fs::canonicalize(forbidden_root).unwrap_or_else(|_| forbidden_root.into());

    if real_parent.starts_with(&real_root) {
        return Err(format!(
            "拒绝写入：{} 位于当前工作目录（{}）之内，{what}不得落入项目目录。\
             请清除 APPDATA / XDG_CONFIG_HOME / HOME 中的异常取值。",
            real_parent.display(),
            real_root.display()
        ));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 注入一组环境变量做解析器。
    ///
    /// 生命周期必须**显式命名**：返回的闭包借用了切片，而 `'_` 在这种
    /// 位置指代不明（E0106）。
    fn env_of<'a>(
        pairs: &'a [(&'static str, &'static str)],
    ) -> impl Fn(&str) -> Option<String> + 'a {
        move |k: &str| {
            pairs
                .iter()
                .find(|(n, _)| *n == k)
                .map(|(_, v)| (*v).to_string())
        }
    }

    /// 路径解析在**搬过来之后**仍然只认那三个变量，且不回退。
    ///
    /// 这条是搬运算不算「行为不变」的判据 —— `gui/config.rs` 那边另有
    /// 一组经包装函数的测试，两侧都过才算数。
    #[test]
    fn resolution_order_and_no_fallback() {
        let p = resolve_user_path(&env_of(&[("APPDATA", "/appdata")]), "d", "f.json");
        assert_eq!(p.unwrap(), Path::new("/appdata").join("d").join("f.json"));

        let p = resolve_user_path(
            &env_of(&[("APPDATA", "/appdata"), ("HOME", "/home/me")]),
            "d",
            "f.json",
        );
        assert_eq!(
            p.unwrap(),
            Path::new("/appdata").join("d").join("f.json"),
            "APPDATA 优先于 HOME"
        );

        let p = resolve_user_path(&env_of(&[("HOME", "/home/me")]), "d", "f.json");
        assert_eq!(
            p.unwrap(),
            Path::new("/home/me")
                .join(".config")
                .join("d")
                .join("f.json")
        );

        // 全都没有 → None。**绝不回退到当前目录或 exe 目录。**
        assert!(resolve_user_path(&env_of(&[]), "d", "f.json").is_none());
    }

    #[test]
    fn blank_env_is_treated_as_absent() {
        let p = resolve_user_path(
            &env_of(&[("APPDATA", "   "), ("HOME", "/home/me")]),
            "d",
            "f.json",
        );
        assert_eq!(
            p.unwrap(),
            Path::new("/home/me")
                .join(".config")
                .join("d")
                .join("f.json"),
            "纯空白不算设置了"
        );
    }

    #[test]
    fn missing_path_degrades_to_default_with_a_note() {
        let (v, note) = load_json::<Vec<u8>>(None);
        assert!(v.is_empty());
        let note = note.expect("必须给出原因");
        assert!(note.contains("仅保存在内存中"), "实测：{note}");
    }

    #[test]
    fn gate_refuses_a_target_inside_the_forbidden_root() {
        let root = std::env::temp_dir().join("scope-persist-test-root");
        let inside = root.join("sub");
        std::fs::create_dir_all(&inside).unwrap();
        let e = check_writable(&inside, &root, "标定数据").unwrap_err();
        assert!(e.contains("拒绝写入"), "实测：{e}");
        assert!(e.contains("不得落入项目目录"), "实测：{e}");
        assert!(e.contains("标定数据"), "`what` 必须出现在拒绝消息里：{e}");
        let _ = std::fs::remove_dir_all(&root);
    }

    /// 临时名必须**从目标文件派生**。
    ///
    /// 判据是把派生出来的那个临时路径**提前占成一个目录** —— 往里写必失败：
    /// - 派生名实现：写 `a.json.tmp` → 撞上目录 → `Err`
    /// - 写死名实现（搬运前的 `config.json.tmp`）：压根不碰 `a.json.tmp` → `Ok`
    ///
    /// 所以 `assert!(e.is_err())` 是这条测试的**全部价值**。
    ///
    /// ⚠ 本测试的第一版是错的：它只断言「`a.json.tmp` 不存在」，
    /// 而那在写死名的实现下**恒真**（那些路径根本不会被创建）。
    /// 对抗性核查把实现变异回写死名，测试照样绿 —— 是变异抓出来的，不是读出来的。
    /// **别把它改回那种写法。**
    #[test]
    fn temp_name_derives_from_the_target() {
        let root = std::env::temp_dir().join("scope-persist-test-derive");
        let _ = std::fs::remove_dir_all(&root);
        std::fs::create_dir_all(root.join("a.json.tmp")).unwrap();

        let e = save_json_atomic(&root.join("a.json"), &vec![1u8], "测试");
        assert!(
            e.is_err(),
            "临时名没有从目标文件派生 —— 写死名会让这条测试变绿，而它本该变红"
        );

        let _ = std::fs::remove_dir_all(&root);
    }

    /// 同目录写两个不同的文件：各自成功，且不留临时文件。
    ///
    /// ⚠ 这条**不能**用来证明「临时名派生」—— 见上面那条的说明。
    /// 它证明的是结果对不对，不是实现怎么做。
    #[test]
    fn saving_two_files_in_one_dir_leaves_no_temp_files() {
        let root = std::env::temp_dir().join("scope-persist-test-tmp");
        let _ = std::fs::remove_dir_all(&root);
        std::fs::create_dir_all(&root).unwrap();

        let a = root.join("a.json");
        let b = root.join("b.json");
        save_json_atomic(&a, &vec![1u8, 2], "测试").unwrap();
        save_json_atomic(&b, &vec![3u8], "测试").unwrap();

        assert_eq!(
            std::fs::read_to_string(&a).unwrap().trim(),
            "[\n  1,\n  2\n]"
        );
        assert_eq!(std::fs::read_to_string(&b).unwrap().trim(), "[\n  3\n]");
        // 临时文件必须已被 rename 走，不能留下
        assert!(!root.join("a.json.tmp").exists(), "残留了临时文件");
        assert!(!root.join("b.json.tmp").exists(), "残留了临时文件");
        let _ = std::fs::remove_dir_all(&root);
    }
}
