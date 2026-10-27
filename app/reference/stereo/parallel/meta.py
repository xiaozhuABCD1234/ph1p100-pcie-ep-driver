#!/usr/bin/env python3
"""证据元信息：固定输出目录的证据需要能判断"是哪版脚本产出的"。

用法（在 runner 里）：
    sys.path.insert(0, str(REPO / "parallel"))
    import meta
    result["evidence_meta"] = meta.evidence_meta(__file__)
"""

from __future__ import annotations

import datetime
import hashlib
from pathlib import Path


def evidence_meta(script_file) -> dict:
    p = Path(script_file)
    return {
        "generated_utc": datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "script": p.name,
        "script_sha256": hashlib.sha256(p.read_bytes()).hexdigest(),
    }
