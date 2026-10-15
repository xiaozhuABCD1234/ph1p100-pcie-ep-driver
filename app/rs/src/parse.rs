// SPDX-License-Identifier: GPL-2.0
//! 原 C 版用的 strtoul(3) / fscanf("%x") 的等价实现（行为对得上，不引 crate）。

/// `strtoul(s, &end, 0)` 的等价：base 0（0x 十六进制 / 0 开头八进制 / 其余十进制）、
/// 允许前导空白与正负号。返回 (解析出的值, 是否整串被吃光)。
///
/// 溢出按 C 的做法饱和到 ULONG_MAX；负号按无符号回绕（`strtoul` 就是这个语义）。
/// 一个数字都没有时返回 (0, false)（C 里 end 停在串首，只有空串才恰好串首即串尾）。
pub fn strtoul_base0_prefix(s: &str) -> (u64, bool) {
    let b = s.as_bytes();
    let mut i = 0;
    while i < b.len() && (b[i] as char).is_ascii_whitespace() {
        i += 1;
    }
    let mut neg = false;
    if i < b.len() && (b[i] == b'+' || b[i] == b'-') {
        neg = b[i] == b'-';
        i += 1;
    }
    let mut base: u64 = 10;
    if i + 1 < b.len() && b[i] == b'0' && (b[i + 1] | 0x20) == b'x' {
        base = 16;
        i += 2;
    } else if i < b.len() && b[i] == b'0' {
        base = 8;
    }

    let mut v: u64 = 0;
    let mut any = false;
    while i < b.len() {
        let d = match (b[i] as char).to_digit(base as u32) {
            Some(d) => d as u64,
            None => break,
        };
        any = true;
        v = v
            .checked_mul(base)
            .and_then(|x| x.checked_add(d))
            .unwrap_or(u64::MAX); // ERANGE：strtoul 返回 ULONG_MAX
        i += 1;
    }
    let consumed = i == b.len();
    let val = if neg { v.wrapping_neg() } else { v };
    (val, consumed && (any || s.is_empty()))
}

/// `strtoul(s, &end, 0)`，且要求整串被吃光（对应 C 里 `*end == '\0'` 的判据）。
/// None = 尾巴上有非数字（C 版就是靠 `*end != '\0'` 判错的）。
pub fn strtoul_base0(s: &str) -> Option<u64> {
    let (v, ok) = strtoul_base0_prefix(s);
    if ok { Some(v) } else { None }
}

/// `fscanf(f, "%llx", &out)` 的等价：跳过前导空白、允许正负号与 0x 前缀，
/// 吃到第一个非十六进制字符为止（**允许尾巴有垃圾**，这正是 fscanf 的行为）。
/// 一个十六进制数字都没读到则返回 None（fscanf 返回 0）。
pub fn hex_prefix_u64(s: &str) -> Option<u64> {
    let b = s.as_bytes();
    let mut i = 0;
    while i < b.len() && (b[i] as char).is_ascii_whitespace() {
        i += 1;
    }
    let mut neg = false;
    if i < b.len() && (b[i] == b'+' || b[i] == b'-') {
        neg = b[i] == b'-';
        i += 1;
    }
    if i + 1 < b.len() && b[i] == b'0' && (b[i + 1] | 0x20) == b'x' {
        i += 2;
    }
    let mut v: u64 = 0;
    let mut any = false;
    while i < b.len() {
        let d = match (b[i] as char).to_digit(16) {
            Some(d) => d as u64,
            None => break,
        };
        any = true;
        v = v.checked_mul(16).and_then(|x| x.checked_add(d)).unwrap_or(u64::MAX);
        i += 1;
    }
    if !any {
        return None;
    }
    Some(if neg { v.wrapping_neg() } else { v })
}

/// `fscanf(f, "%x", &unsigned)` 的等价（赋给 32 位无符号，截断）
pub fn hex_prefix_u32(s: &str) -> Option<u32> {
    hex_prefix_u64(s).map(|v| v as u32)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn strtoul_like_c() {
        assert_eq!(strtoul_base0("0x40"), Some(64));
        assert_eq!(strtoul_base0("010"), Some(8));
        assert_eq!(strtoul_base0("1000"), Some(1000));
        assert_eq!(strtoul_base0("  0x5c "), None);
        assert_eq!(strtoul_base0("0x5c "), None);
        assert_eq!(strtoul_base0("abc"), None);
        assert_eq!(strtoul_base0(""), Some(0)); // strtoul("")：end 停在串首，*end==0
        assert_eq!(strtoul_base0("0x"), None); // 解析到 "0"，end 指向 'x'
        assert_eq!(strtoul_base0("0xffffffff"), Some(0xffff_ffff));
        assert_eq!(strtoul_base0("0x100000000"), Some(0x1_0000_0000));
        // 不检查 end 的用法（pci_bar 的 strtoul）：尾巴有垃圾也拿到前缀值
        assert_eq!(strtoul_base0_prefix("1abc"), (1, false));
        assert_eq!(strtoul_base0_prefix("0x1g"), (1, false));
        assert_eq!(strtoul_base0_prefix("0x100000001").0 as u32, 1);
    }

    #[test]
    fn hex_like_fscanf() {
        assert_eq!(hex_prefix_u32("0x1edb\n"), Some(0x1edb));
        assert_eq!(hex_prefix_u32("abcd\n"), Some(0xabcd));
        assert_eq!(hex_prefix_u32(" 10 \n"), Some(0x10));
        assert_eq!(hex_prefix_u32("xyz"), None);
        assert_eq!(hex_prefix_u64("00000000f0000000"), Some(0xf000_0000));
        assert_eq!(hex_prefix_u64("0x040200"), Some(0x04_0200));
    }
}
