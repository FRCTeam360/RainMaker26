"""ntlive.py — live NetworkTables (NT4) client for AdvantageKit telemetry.

Library + CLI. Connects to the sim (127.0.0.1) or a real robot (read-only), decodes
AdvantageKit struct values (Pose2d, ChassisSpeeds, ...), and evaluates live checks.

CLI:
  topics  [--filter SUBSTR]                 list topics and types
  get     KEY [KEY...]                      print latest values as JSON
  watch   KEY [KEY...] --duration S --hz N  stream JSONL samples
  record  --out FILE.wpilog --duration S    record every topic to a WPILOG
  check   --spec FILE.json --duration S     evaluate checks, exit 0 on pass / 1 on fail

KEY forms: "RealOutputs/Swerve/CurrentPose" (prefixed with /AdvantageKit/), a full
"/SmartDashboard/..." path, and an optional field path after "::",
e.g. "RealOutputs/Swerve/CurrentPose::translation.x".
"""

from __future__ import annotations

import argparse
import json
import math
import re
import struct
import sys
import time

import ntcore

AKIT_PREFIX = "/AdvantageKit/"
SCHEMA_PREFIXES = ["/AdvantageKit/.schema/", "/.schema/"]
SIM_HOST = "127.0.0.1"
NT_PORT = 5810

# ── Key handling ─────────────────────────────────────────────────────────────


def split_key(key: str) -> tuple[str, list[str]]:
    """Returns (full topic name, field path)."""
    name, _, field = key.partition("::")
    if not name.startswith("/"):
        name = AKIT_PREFIX + name
    return name, [f for f in field.split(".") if f]


def dig(value, path: list[str]):
    for part in path:
        if isinstance(value, list):
            value = value[int(part)]
        else:
            value = value[part]
    return value


# ── Struct decoding ──────────────────────────────────────────────────────────

PRIMITIVES = {
    "bool": "?", "char": "c", "int8": "b", "int16": "h", "int32": "i", "int64": "q",
    "uint8": "B", "uint16": "H", "uint32": "I", "uint64": "Q",
    "float": "f", "float32": "f", "double": "d", "float64": "d",
}
DECL_RE = re.compile(r"^(?:enum\s*\{[^}]*\}\s*)?(\w+)\s+(\w+)\s*(?:\[(\d+)\])?$")


class StructRegistry:
    def __init__(self):
        self.schemas: dict[str, list[tuple[str, str, int | None]]] = {}

    def add(self, type_name: str, schema_text: str):
        fields = []
        for decl in schema_text.split(";"):
            decl = decl.strip()
            if not decl:
                continue
            m = DECL_RE.match(decl)
            if not m:  # bit-fields and other exotic declarations are not supported
                return
            typ, name, arr = m.groups()
            fields.append((typ, name, int(arr) if arr else None))
        self.schemas[type_name] = fields

    def size(self, typ: str) -> int | None:
        if typ in PRIMITIVES:
            return struct.calcsize("<" + PRIMITIVES[typ])
        if typ not in self.schemas:
            return None
        total = 0
        for ftyp, _, arr in self.schemas[typ]:
            s = self.size(ftyp)
            if s is None:
                return None
            total += s * (arr or 1)
        return total

    def _decode_one(self, typ: str, buf: bytes, off: int):
        if typ in PRIMITIVES:
            fmt = "<" + PRIMITIVES[typ]
            return struct.unpack_from(fmt, buf, off)[0], off + struct.calcsize(fmt)
        out = {}
        for ftyp, name, arr in self.schemas[typ]:
            if arr is None:
                out[name], off = self._decode_one(ftyp, buf, off)
            elif ftyp == "char":
                out[name] = buf[off:off + arr].split(b"\0", 1)[0].decode(errors="replace")
                off += arr
            else:
                items = []
                for _ in range(arr):
                    item, off = self._decode_one(ftyp, buf, off)
                    items.append(item)
                out[name] = items
        return out, off

    def decode(self, type_string: str, raw: bytes):
        """Decodes 'struct:X' or 'struct:X[]'. Returns None if the schema is unknown."""
        if not type_string.startswith("struct:"):
            return None
        typ = type_string[len("struct:"):]
        is_array = typ.endswith("[]")
        typ = typ.removesuffix("[]")
        size = self.size(typ)
        if not size:
            return None
        if is_array:
            return [self._decode_one(typ, raw, i)[0] for i in range(0, len(raw) - size + 1, size)]
        if len(raw) < size:
            return None
        return self._decode_one(typ, raw, 0)[0]


# ── Client ───────────────────────────────────────────────────────────────────


class LiveNT:
    """Caches the latest value of subscribed topics. Call pump() often."""

    def __init__(self, host: str = SIM_HOST, port: int = NT_PORT, name: str = "claude-ntlive"):
        self.host = host
        self.inst = ntcore.NetworkTableInstance.create()
        self.inst.startClient4(name)
        self.inst.setServer(host, port)
        # NT4 only announces topics that match a subscription; this makes every topic visible.
        self._announce_sub = ntcore.MultiSubscriber(self.inst, ["/"], ntcore.PubSubOptions(topicsOnly=True))
        self.structs = StructRegistry()
        self.latest: dict[str, tuple[object, str, float]] = {}  # name -> (value, type, local time)
        self.update_counts: dict[str, int] = {}
        self.exact: set[str] = set()
        self.prefixes: list[str] = []
        self.poller = ntcore.NetworkTableListenerPoller(self.inst)
        self.poller.addListener(SCHEMA_PREFIXES, ntcore.EventFlags.kValueAll | ntcore.EventFlags.kImmediate)
        self._publishers = {}

    def wait_connected(self, timeout: float = 10.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.inst.isConnected():
                return True
            time.sleep(0.1)
        return False

    def wait_topic(self, name: str, timeout: float = 30.0) -> bool:
        name = split_key(name)[0]
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.inst.getTopic(name).exists():
                return True
            time.sleep(0.2)
        return False

    def subscribe(self, keys=(), prefixes=()):
        names = {split_key(k)[0] for k in keys} - self.exact
        new_prefixes = [p for p in prefixes if p not in self.prefixes]
        mask = ntcore.EventFlags.kValueAll | ntcore.EventFlags.kImmediate
        if names:
            self.exact |= names
            self.poller.addListener(sorted(names), mask)
        if new_prefixes:
            self.prefixes += new_prefixes
            self.poller.addListener(new_prefixes, mask)

    def pump(self):
        for ev in self.poller.readQueue():
            data = ev.data
            if not isinstance(data, ntcore.ValueEventData):
                continue
            name = data.topic.getName()
            type_string = data.topic.getTypeString()
            val = data.value.value()
            if "/.schema/struct:" in name:
                self.structs.add(name.rsplit("struct:", 1)[1], bytes(val).decode(errors="replace"))
                continue
            if name not in self.exact and not any(name.startswith(p) for p in self.prefixes):
                continue  # prefix listener matched a longer name
            self.latest[name] = (val, type_string, time.monotonic())
            self.update_counts[name] = self.update_counts.get(name, 0) + 1

    def value(self, key: str):
        """Latest decoded value for KEY (with optional ::field path), or None if never received."""
        name, path = split_key(key)
        entry = self.latest.get(name)
        if entry is None:
            return None
        val, type_string, _ = entry
        if isinstance(val, (bytes, bytearray)):
            decoded = self.structs.decode(type_string, bytes(val))
            val = decoded if decoded is not None else {"_raw_hex": bytes(val).hex(), "_type": type_string}
        return dig(val, path) if path else val

    def age(self, key: str) -> float:
        entry = self.latest.get(split_key(key)[0])
        return math.inf if entry is None else time.monotonic() - entry[2]

    def topics(self, filt: str = ""):
        return sorted(
            ((t.getName(), t.getTypeString()) for t in self.inst.getTopics() if filt.lower() in t.getName().lower()),
        )

    def publish_string(self, name: str, value: str):
        """Only used against the simulator (guarded by callers)."""
        if self.host not in (SIM_HOST, "localhost"):
            raise RuntimeError("refusing to publish to a non-simulator NT server")
        pub = self._publishers.get(name)
        if pub is None:
            pub = self._publishers[name] = self.inst.getStringTopic(name).publish()
        pub.set(value)

    def start_recording(self, path: str):
        """Records every NT topic to a WPILOG (readable by AdvantageScope / wpilog-mcp)."""
        from wpiutil import DataLogBackgroundWriter

        log = DataLogBackgroundWriter("", path, 0.25)
        self.inst.startEntryDataLog(log, "", "NT:")
        # Subscribe to everything so the server sends every value.
        self._record_sub = ntcore.MultiSubscriber(self.inst, ["/"], ntcore.PubSubOptions(sendAll=True))
        return log

    def stop_recording(self, log):
        # stopEntryDataLog() deadlocks under pyntcore 2026.2.2; stopping the writer is enough,
        # and close() tears down the entry logger with the instance.
        log.flush()
        log.stop()

    def close(self):
        ntcore.NetworkTableInstance.destroy(self.inst)


# ── Checks ───────────────────────────────────────────────────────────────────
#
# Spec: {"checks": [ {"name": str, "kind": "always"|"never"|"eventually"|"final",
#                     "expr": str, "after": s (optional), "within": s (optional)} ]}
# Expressions are Python, evaluated each tick with:
#   v(key)        latest value (None if never received)
#   start(key)    value at the first tick where it existed
#   age(key)      seconds since the key last updated
#   t             seconds since the check window opened
#   dist(a, b)    distance between two Pose2d/Translation2d-like dicts
#   abs min max len any all hypot math
# "always"/"never" are only evaluated inside [after, within]; if an expression raises
# (e.g. key missing) at the end of the window the check fails with "no data".

KEY_CALL_RE = re.compile(r"""\b(?:v|start|age)\(\s*['"]([^'"]+)['"]\s*\)""")


def _xy(p):
    if isinstance(p, dict) and "translation" in p:
        p = p["translation"]
    return p["x"], p["y"]


def dist(a, b):
    (ax, ay), (bx, by) = _xy(a), _xy(b)
    return math.hypot(ax - bx, ay - by)


class CheckRunner:
    def __init__(self, nt: LiveNT, spec: dict):
        self.nt = nt
        self.checks = spec.get("checks", [])
        keys = {k for c in self.checks for k in KEY_CALL_RE.findall(c["expr"])}
        nt.subscribe(keys=keys)
        self.starts: dict[str, object] = {}
        self.t0 = None
        self.state = [
            {"name": c["name"], "kind": c["kind"], "expr": c["expr"], "passed": None, "detail": "",
             "evaluated": 0, "errors": 0, "first_true_t": None, "last_value": None}
            for c in self.checks
        ]

    def _env(self, t):
        def start(key):
            if key not in self.starts:
                val = self.nt.value(key)
                if val is None:
                    raise KeyError(f"no data for {key}")
                self.starts[key] = val
            return self.starts[key]

        def v(key):
            val = self.nt.value(key)
            if val is None:
                raise KeyError(f"no data for {key}")
            return val

        return {"v": v, "start": start, "age": self.nt.age, "t": t, "dist": dist,
                "abs": abs, "min": min, "max": max, "len": len, "any": any, "all": all,
                "hypot": math.hypot, "math": math}

    def tick(self):
        now = time.monotonic()
        if self.t0 is None:
            self.t0 = now
        t = now - self.t0
        env = self._env(t)
        for c, s in zip(self.checks, self.state):
            if s["passed"] is not None:  # verdict already reached
                continue
            if t < c.get("after", 0) or ("within" in c and t > c["within"]):
                continue
            try:
                result = bool(eval(c["expr"], {"__builtins__": {}}, env))
            except Exception as e:  # noqa: BLE001 — missing data is a normal early condition
                s["errors"] += 1
                s["detail"] = f"{type(e).__name__}: {e}"
                continue
            s["evaluated"] += 1
            s["last_value"] = result
            if result and s["first_true_t"] is None:
                s["first_true_t"] = round(t, 3)
            if s["kind"] == "always" and not result:
                s["passed"], s["detail"] = False, f"violated at t={t:.2f}s"
            elif s["kind"] == "never" and result:
                s["passed"], s["detail"] = False, f"occurred at t={t:.2f}s"
            elif s["kind"] == "eventually" and result:
                s["passed"], s["detail"] = True, f"true at t={t:.2f}s"

    def finish(self) -> list[dict]:
        for s in self.state:
            if s["passed"] is not None:
                continue
            if s["evaluated"] == 0:
                s["passed"] = False
                s["detail"] = f"never evaluated (no data) — last error: {s['detail']}"
            elif s["kind"] in ("always", "never"):
                s["passed"], s["detail"] = True, f"held for {s['evaluated']} samples"
            elif s["kind"] == "eventually":
                s["passed"], s["detail"] = False, "never became true"
            elif s["kind"] == "final":
                s["passed"] = bool(s["last_value"])
                s["detail"] = f"final value {s['last_value']}"
        return self.state


def run_checks(nt: LiveNT, spec: dict, duration: float, hz: float = 50.0, should_stop=None) -> list[dict]:
    runner = CheckRunner(nt, spec)
    end = time.monotonic() + duration
    while time.monotonic() < end and not (should_stop and should_stop()):
        nt.pump()
        runner.tick()
        time.sleep(1.0 / hz)
    nt.pump()
    runner.tick()
    return runner.finish()


def print_results(results: list[dict]) -> bool:
    ok = all(r["passed"] for r in results)
    for r in results:
        print(f"  [{'PASS' if r['passed'] else 'FAIL'}] {r['name']} ({r['kind']}): {r['detail']}")
    print(f"RESULT: {'PASS' if ok else 'FAIL'} ({sum(r['passed'] for r in results)}/{len(results)})")
    return ok


# ── CLI ──────────────────────────────────────────────────────────────────────


def _jsonable(x):
    if isinstance(x, (bytes, bytearray)):
        return bytes(x).hex()
    if isinstance(x, float) and not math.isfinite(x):
        return str(x)
    return x


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default=SIM_HOST, help="NT server (sim: 127.0.0.1, robot: 10.3.60.2)")
    ap.add_argument("--port", type=int, default=NT_PORT)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("topics"); p.add_argument("--filter", default="")
    p = sub.add_parser("get"); p.add_argument("keys", nargs="+")
    p = sub.add_parser("watch"); p.add_argument("keys", nargs="+")
    p.add_argument("--duration", type=float, default=10); p.add_argument("--hz", type=float, default=10)
    p = sub.add_parser("record"); p.add_argument("--out", required=True)
    p.add_argument("--duration", type=float, default=30)
    p = sub.add_parser("check"); p.add_argument("--spec", required=True)
    p.add_argument("--duration", type=float, default=15); p.add_argument("--json", action="store_true")
    args = ap.parse_args(argv)

    nt = LiveNT(args.host, args.port)
    if not nt.wait_connected():
        print(f"ERROR: could not connect to NT server at {args.host}:{args.port} — is the robot/sim running?", file=sys.stderr)
        return 2
    time.sleep(0.5)  # let topic announcements arrive

    if args.cmd == "topics":
        for name, typ in nt.topics(args.filter):
            print(f"{typ:28} {name}")
    elif args.cmd == "get":
        nt.subscribe(keys=args.keys)
        time.sleep(0.5)
        nt.pump()
        print(json.dumps({k: nt.value(k) for k in args.keys}, indent=2, default=_jsonable))
    elif args.cmd == "watch":
        nt.subscribe(keys=args.keys)
        t0 = time.monotonic()
        while time.monotonic() - t0 < args.duration:
            nt.pump()
            row = {"t": round(time.monotonic() - t0, 3), **{k: nt.value(k) for k in args.keys}}
            print(json.dumps(row, default=_jsonable), flush=True)
            time.sleep(1.0 / args.hz)
    elif args.cmd == "record":
        rec = nt.start_recording(args.out)
        time.sleep(args.duration)
        nt.stop_recording(rec)
        print(f"wrote {args.out}")
    elif args.cmd == "check":
        with open(args.spec) as f:
            spec = json.load(f)
        results = run_checks(nt, spec, args.duration)
        if args.json:
            print(json.dumps(results, indent=2))
        ok = print_results(results)
        nt.close()
        return 0 if ok else 1
    nt.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
