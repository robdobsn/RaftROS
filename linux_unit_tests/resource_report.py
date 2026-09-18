import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shlex
import struct
import subprocess


def command(arguments, directory=None):
    return subprocess.check_output(arguments, cwd=directory, text=True, timeout=30,
                                   env={**os.environ, "LC_ALL": "C"}).strip()


def stack_entries(directory):
    entries = []
    for path in sorted(directory.rglob("*.su")):
        for line in path.read_text().splitlines():
            fields = line.split("\t")
            if len(fields) != 3:
                raise ValueError(f"Unexpected stack-usage record in {path}: {line}")
            entries.append({"function": fields[0], "bytes": int(fields[1]), "kind": fields[2],
                            "file": str(path.relative_to(directory))})
    if not entries:
        raise ValueError(f"No compiler stack-usage records in {directory}")
    return sorted(entries, key=lambda entry: (-entry["bytes"], entry["function"]))


def artifact(path, build_dir, size_tool):
    with path.open("rb") as binary:
        header = binary.read(20)
    if header[:4] != b"\x7fELF" or header[4] not in (1, 2) or header[5] not in (1, 2):
        raise ValueError(f"Expected a supported ELF executable: {path}")
    byte_order = "<" if header[5] == 1 else ">"
    elf_machine = struct.unpack_from(byte_order + "H", header, 18)[0]
    sections = command(shlex.split(size_tool) + ["-B", str(path)]).splitlines()
    if len(sections) != 2:
        raise ValueError(f"Unexpected size output for {path}: {sections}")
    text, data, bss = (int(field) for field in sections[1].split()[:3])
    symbols = command(["nm", "-C", "--defined-only", str(path)])
    map_path = path.with_name(path.name + ".map")
    if not map_path.is_file():
        raise ValueError(f"Missing linker map: {map_path}")
    stacks = stack_entries(build_dir)
    return {
        "executable": str(path),
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "file_bytes": path.stat().st_size,
        "elf_class_bits": 32 if header[4] == 1 else 64,
        "elf_machine_id": elf_machine,
        "endianness": "little" if header[5] == 1 else "big",
        "sections": {"text": text, "data": data, "bss": bss,
                     "text_plus_data": text + data, "data_plus_bss": data + bss},
        "linker_map": str(map_path),
        "largest_compiled_frames": stacks[:8],
        "main_frames": [entry for entry in stacks if ":int main(" in entry["function"]],
        "storage_constructor_frames": [entry for entry in stacks if "ProbeStorage::ProbeStorage(" in entry["function"]],
        "session_start_frames": [entry for entry in stacks if "ZenohTCPSession::start(" in entry["function"]],
        "defined_symbol_presence": {
            "zenoh": "RaftRuntime::Zenoh::" in symbols,
            "rtps_wire": any(name in symbols for name in
                             ("RTPSMessage::", "RTPSParticipant::", "SPDPHandler::", "SEDPHandler::")),
            "sensor_serializer": "RaftRuntime::AutoPub::AutoPubCDRSerializer_serialize(" in symbols,
        },
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path("build/resources"))
    parser.add_argument("--compiler", default="g++")
    parser.add_argument("--size-tool", default="size")
    parser.add_argument("--compile-flags", required=True)
    parser.add_argument("--max-probe-main-frame", type=int, default=4096)
    args = parser.parse_args()
    root = args.root.resolve(strict=True)
    workspace = Path(__file__).resolve().parent
    rtps = artifact(root / "rtps/raftros_linux", root / "rtps", args.size_tool)
    zenoh = artifact(root / "zenoh/zenoh/zenoh_session_probe", root / "zenoh", args.size_tool)
    if rtps["defined_symbol_presence"]["zenoh"] or zenoh["defined_symbol_presence"]["rtps_wire"]:
        raise ValueError("Unexpected backend wire symbols in isolated resource builds")
    if not zenoh["defined_symbol_presence"]["sensor_serializer"]:
        raise ValueError("Zenoh probe must link the common sensor serializer")
    storage = json.loads(command([zenoh["executable"], "--resource-info"]))
    component_bytes = sum(storage[name] for name in
                          ("session_bytes", "publication_probe_bytes", "socket_rx_scratch_bytes"))
    if (storage["storage_location"] != "single_heap_allocation" or
            not component_bytes <= storage["owned_storage_bytes"] <= storage["owned_storage_limit_bytes"]):
        raise ValueError("Probe persistent storage must use one bounded off-stack owner")
    main_frames = zenoh["main_frames"]
    if (args.max_probe_main_frame <= 0 or len(main_frames) != 1 or
            main_frames[0]["kind"] not in ("static", "dynamic,bounded") or
            main_frames[0]["bytes"] > args.max_probe_main_frame):
        raise ValueError(f"Probe main stack frame exceeds {args.max_probe_main_frame} bytes or is unbounded/missing")
    report = {
        "measured_at_utc": datetime.now(timezone.utc).isoformat(),
        "compiler": command(shlex.split(args.compiler) + ["--version"]).splitlines()[0],
        "compiler_target": command(shlex.split(args.compiler) + ["-dumpmachine"]),
        "binutils": command(shlex.split(args.size_tool) + ["--version"]).splitlines()[0],
        "compile_flags": args.compile_flags,
        "link_flags": "-Wl,--gc-sections,-Map=<per-artifact path>",
        "repository_revision": command(["git", "rev-parse", "HEAD"], workspace),
        "worktree_status": command(["git", "status", "--short"], workspace),
        "raftcore_revision": command(["git", "rev-parse", "HEAD"], workspace / "RaftCore"),
        "rtps": rtps,
        "zenoh": zenoh,
        "zenoh_fixed_storage": storage,
        "probe_main_frame_limit_bytes": args.max_probe_main_frame,
        "limits": [
            "Host executables are not feature-equivalent; neither is a firmware transport footprint.",
            "ELF sections exclude shared-library memory and code, heap, thread stacks and kernel socket storage.",
            "File size includes ELF metadata and is not text+data or flash image size.",
            "Compiler stack records include discarded functions; per-function frames are not call-chain maxima.",
            "Probe persistent storage is one fixed heap allocation, separate from reported stack and BSS; allocator overhead is not measured.",
            "The main-frame check is a host regression guard, not a bound on nested calls or an ESP32 task high-water mark.",
            "No runtime RSS/heap/CPU, live ESP32 memory, or OTA image measurement is made by this report.",
        ],
    }
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()