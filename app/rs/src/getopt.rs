// SPDX-License-Identifier: GPL-2.0
//! `getopt_long(3)` 的等价实现（GNU 语义），做成迭代器。
//!
//! 原 C 版三个工具都用 getopt_long 解析参数，行为要点：
//!   * 长选项可无歧义前缀缩写（`--sel` = `--selftest`）；
//!   * 长选项 `--name=value` 与 `--name value` 都行；
//!   * 短选项可连写（`-nt100`），带参短选项的参数可紧跟其后；
//!   * 选项与位置参数可交错（GNU 会重排）；
//!   * `--` 之后一律当位置参数；单独一个 `-` 也是位置参数；
//!   * 与 C 一样是「一次交还一个选项」：调用方遇到 -h 就直接返回，后面那些
//!     没解析到的参数不会被报错（`-hx` 在原版也是打印用法后以 0 退出）。
//!
//! 出错时返回的消息文本与 glibc 一致（由调用方按 `argv[0]: 消息` 打到 stderr）。

/// 对应 C 版 `struct option` 的一项。
#[derive(Clone, Copy)]
pub struct LongOpt {
    pub name: &'static str,
    /// 是否需要参数（对应 `required_argument`）
    pub has_arg: bool,
    /// 对应 C 版 `struct option` 的 `val`：命中后返回给调用方的编号
    /// （短选项用该字符的 ASCII 值）
    pub val: u32,
}

fn short_lookup(shortopts: &str, c: char) -> Option<bool> {
    // 短选项描述串形如 "s:d:t:h"：字符后跟 ':' 表示「需要参数」
    let chars: Vec<char> = shortopts.chars().collect();
    let mut i = 0;
    while i < chars.len() {
        if chars[i] == ':' {
            i += 1;
            continue;
        }
        let needs = i + 1 < chars.len() && chars[i + 1] == ':';
        if chars[i] == c {
            return Some(needs);
        }
        i += if needs { 2 } else { 1 };
    }
    None
}

/// 一次解析结果：编号 +（可能的）参数；Err 是 getopt 风格的错误消息。
pub type OptItem = Result<(u32, Option<String>), String>;

/// `getopt_long(argv, shortopts, longopts, NULL)`；`argv` 含 argv[0]。
pub struct Getopt<'a> {
    argv: &'a [String],
    shortopts: &'a str,
    longopts: &'a [LongOpt],
    /// 下一个待取用的 argv 下标
    i: usize,
    /// 当前短选项簇里剩下的字符
    cluster: Vec<char>,
    cluster_pos: usize,
    no_more_opts: bool,
    positional: Vec<String>,
}

impl<'a> Getopt<'a> {
    pub fn new(argv: &'a [String], shortopts: &'a str, longopts: &'a [LongOpt]) -> Getopt<'a> {
        Getopt {
            argv,
            shortopts,
            longopts,
            i: 1, // 跳过 argv[0]
            cluster: Vec::new(),
            cluster_pos: 0,
            no_more_opts: false,
            positional: Vec::new(),
        }
    }

    /// 到目前为止收集到的位置参数（GNU 重排后它们在选项之后，顺序保持原样）
    pub fn positional(self) -> Vec<String> {
        self.positional
    }

    /// 取下一个 argv（给需要参数的选项用）
    fn take_arg(&mut self) -> Option<String> {
        if self.i < self.argv.len() {
            let v = self.argv[self.i].clone();
            self.i += 1;
            Some(v)
        } else {
            None
        }
    }
}

impl Iterator for Getopt<'_> {
    /// Ok((编号, 参数)) / Err(getopt 风格的错误消息)
    type Item = OptItem;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            // 先把当前短选项簇里的剩余字符发一个出去（每个分支都直接 return，
            // 所以这里是 if 而不是 while；连写的剩余字符由外层 loop 再次进来处理）
            if self.cluster_pos < self.cluster.len() {
                let c = self.cluster[self.cluster_pos];
                match short_lookup(self.shortopts, c) {
                    None => {
                        self.cluster_pos += 1;
                        return Some(Err(format!("invalid option -- '{}'", c)));
                    }
                    Some(false) => {
                        self.cluster_pos += 1;
                        return Some(Ok((c as u32, None)));
                    }
                    Some(true) => {
                        let rest: String = self.cluster[self.cluster_pos + 1..].iter().collect();
                        self.cluster_pos = self.cluster.len();
                        if !rest.is_empty() {
                            return Some(Ok((c as u32, Some(rest))));
                        }
                        return Some(match self.take_arg() {
                            Some(v) => Ok((c as u32, Some(v))),
                            None => Err(format!("option requires an argument -- '{}'", c)),
                        });
                    }
                }
            }
            self.cluster.clear();
            self.cluster_pos = 0;

            if self.i >= self.argv.len() {
                return None;
            }
            let a = self.argv[self.i].clone();
            self.i += 1;

            if self.no_more_opts || a == "-" || !a.starts_with('-') {
                self.positional.push(a);
                continue;
            }
            if a == "--" {
                self.no_more_opts = true;
                continue;
            }

            if let Some(body) = a.strip_prefix("--") {
                // ---- 长选项 ----
                let (name, inline_arg) = match body.find('=') {
                    Some(p) => (&body[..p], Some(body[p + 1..].to_string())),
                    None => (body, None),
                };
                let exact = self.longopts.iter().find(|o| o.name == name);
                let opt = match exact {
                    Some(o) => Some(o),
                    None => {
                        let cands: Vec<&LongOpt> = self
                            .longopts
                            .iter()
                            .filter(|o| o.name.starts_with(name))
                            .collect();
                        match cands.len() {
                            1 => Some(cands[0]),
                            0 => return Some(Err(format!("unrecognized option '--{}'", body))),
                            _ => {
                                let list = cands
                                    .iter()
                                    .map(|o| format!("'--{}'", o.name))
                                    .collect::<Vec<_>>()
                                    .join(" ");
                                return Some(Err(format!(
                                    "option '--{}' is ambiguous; possibilities: {}",
                                    body, list
                                )));
                            }
                        }
                    }
                };
                let opt = opt.expect("上面三个分支必有一个命中");
                if opt.has_arg {
                    let val = match inline_arg {
                        Some(v) => Some(v),
                        None => self.take_arg(),
                    };
                    return Some(match val {
                        Some(v) => Ok((opt.val, Some(v))),
                        None => Err(format!("option '--{}' requires an argument", opt.name)),
                    });
                }
                if inline_arg.is_some() {
                    return Some(Err(format!("option '--{}' doesn't allow an argument", opt.name)));
                }
                return Some(Ok((opt.val, None)));
            }

            // ---- 短选项（可能连写）：交给上面那个 while 逐字发 ----
            self.cluster = a[1..].chars().collect();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn s(v: &[&str]) -> Vec<String> {
        v.iter().map(|x| x.to_string()).collect()
    }

    const OPTS: &[LongOpt] = &[
        LongOpt { name: "source", has_arg: true, val: b's' as u32 },
        LongOpt { name: "timeout", has_arg: true, val: b't' as u32 },
        LongOpt { name: "selftest", has_arg: false, val: 2 },
        LongOpt { name: "stamp", has_arg: false, val: 3 },
        LongOpt { name: "help", has_arg: false, val: b'h' as u32 },
    ];

    fn collect(argv: &[String]) -> (Vec<(u32, Option<String>)>, Vec<String>, Vec<String>) {
        let mut g = Getopt::new(argv, "s:t:h", OPTS);
        let mut opts = Vec::new();
        let mut errs = Vec::new();
        while let Some(item) = g.next() {
            match item {
                Ok(o) => opts.push(o),
                Err(e) => errs.push(e),
            }
        }
        (opts, errs, g.positional())
    }

    #[test]
    fn basics() {
        let (o, e, p) = collect(&s(&["cam_grab", "-s", "cam", "-t", "100", "out.pgm"]));
        assert!(e.is_empty());
        assert_eq!(o, vec![(b's' as u32, Some("cam".into())), (b't' as u32, Some("100".into()))]);
        assert_eq!(p, vec!["out.pgm"]);

        // 连写 + 参数紧跟
        let (o, _, _) = collect(&s(&["x", "-sdispatch"]));
        assert_eq!(o, vec![(b's' as u32, Some("dispatch".into()))]);
        let (o, _, _) = collect(&s(&["x", "-ht100"]));
        assert_eq!(o, vec![(b'h' as u32, None), (b't' as u32, Some("100".into()))]);

        // 长选项前缀缩写 / = 形式
        let (o, _, _) = collect(&s(&["x", "--so=pattern", "--timeout", "5"]));
        assert_eq!(o, vec![(b's' as u32, Some("pattern".into())), (b't' as u32, Some("5".into()))]);

        // 位置参数与选项交错
        let (o, _, p) = collect(&s(&["x", "a.pgm", "--selftest", "-s", "cam", "b"]));
        assert_eq!(o, vec![(2, None), (b's' as u32, Some("cam".into()))]);
        assert_eq!(p, vec!["a.pgm", "b"]);

        // -- 之后都是位置参数
        let (o, _, p) = collect(&s(&["x", "--", "--selftest"]));
        assert!(o.is_empty());
        assert_eq!(p, vec!["--selftest"]);

        // 单独一个 "-" 是位置参数
        let (_, _, p) = collect(&s(&["x", "-"]));
        assert_eq!(p, vec!["-"]);
    }

    #[test]
    fn lazy_like_getopt() {
        // C 版在 -h 处就直接返回了，后面的 'x' 根本不会被 getopt 报错
        let argv = s(&["x", "-hx"]);
        let mut g = Getopt::new(&argv, "s:t:h", OPTS);
        assert_eq!(g.next().unwrap().unwrap(), (b'h' as u32, None));
    }

    #[test]
    fn errors() {
        let (_, e, _) = collect(&s(&["x", "--nope"]));
        assert_eq!(e, vec!["unrecognized option '--nope'".to_string()]);
        let (_, e, _) = collect(&s(&["x", "--s"]));
        assert_eq!(
            e,
            vec!["option '--s' is ambiguous; possibilities: '--source' '--selftest' '--stamp'"
                .to_string()]
        );
        let (_, e, _) = collect(&s(&["x", "--source"]));
        assert_eq!(e, vec!["option '--source' requires an argument".to_string()]);
        let (_, e, _) = collect(&s(&["x", "--help=1"]));
        assert_eq!(e, vec!["option '--help' doesn't allow an argument".to_string()]);
        let (_, e, _) = collect(&s(&["x", "-z"]));
        assert_eq!(e, vec!["invalid option -- 'z'".to_string()]);
        let (_, e, _) = collect(&s(&["x", "-t"]));
        assert_eq!(e, vec!["option requires an argument -- 't'".to_string()]);
    }
}
