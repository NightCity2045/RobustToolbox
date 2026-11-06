#!/usr/bin/env python3
"""Package the trusted engine client, native dependencies and .NET runtime.

Content owns composition with its matching Content.Client.zip. This does not
change launcher packages or disable content verification. Run after tests: dotnet
publish still uses the checkout's intermediate outputs, though it never wipes bin.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET
import zipfile


ROOT = Path(__file__).resolve().parents[1]
TARGETS = {"linux-x64": "Linux", "win-x64": "Windows"}


def git(*args):
    return subprocess.check_output(["git", "-C", str(ROOT), *args], text=True).strip()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime", required=True, choices=TARGETS)
    parser.add_argument("--output", type=Path, required=True, help="New destination ZIP; existing files are refused")
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists():
        parser.error(f"Output already exists: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    source_commit = git("rev-parse", "HEAD")
    dirty = bool(git("status", "--porcelain", "--untracked-files=normal"))
    sdk = subprocess.check_output(["dotnet", "--version"], cwd=ROOT, text=True).strip()
    packages = ET.parse(ROOT / "Directory.Packages.props")
    physics = packages.find(".//PackageVersion[@Include='BepuPhysics']").attrib["Version"]
    protocol = json.loads((ROOT / "Robust.Shared/Spatial/spatial-protocol.json").read_text(encoding="utf-8"))
    if protocol["physics_backend"] != "BepuPhysics" or protocol["physics_version"] != physics:
        raise RuntimeError("Spatial protocol descriptor does not match the pinned physics dependency")
    canonical_protocol = "".join(f"{key}={protocol[key]}\n" for key in sorted(protocol))
    protocol_fingerprint = hashlib.sha256(canonical_protocol.encode("utf-8")).hexdigest().upper()

    with tempfile.TemporaryDirectory(prefix="robust-standalone-") as temporary:
        stage = Path(temporary) / "client"
        subprocess.run([
            "dotnet", "publish", "Robust.Client/Robust.Client.csproj", "-c", "Release",
            "--runtime", args.runtime, "--self-contained", "true", "-m:2",
            "/p:FullRelease=True", "/p:UseAppHost=True", f"/p:TargetOS={TARGETS[args.runtime]}",
            "--output", str(stage)
        ], cwd=ROOT, check=True)
        shutil.copytree(ROOT / "Resources", stage / "Resources",
                        ignore=shutil.ignore_patterns(".gitignore", ".directory", ".DS_Store"))
        executable = "Robust.Client.exe" if args.runtime.startswith("win-") else "Robust.Client"
        # Catch omission of the runtime or backend before producing an apparently usable archive.
        required = [executable, "Robust.Client.dll", "BepuPhysics.dll", "BepuUtilities.dll",
                    "System.Private.CoreLib.dll"]
        for name in required:
            if not (stage / name).is_file():
                raise RuntimeError(f"Publish omitted required file: {name}")
        files = {path.relative_to(stage).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest()
                 for path in sorted(stage.rglob("*")) if path.is_file()}
        manifest = {"schema_version": 1, "kind": "standalone-engine-client",
                    "engine_commit": source_commit, "source_dirty": dirty,
                    "runtime": args.runtime, "sdk": sdk, "physics_backend": "BepuPhysics",
                    "physics_version": physics, "spatial_protocol": protocol,
                    "spatial_protocol_fingerprint": protocol_fingerprint, "files": files}
        (stage / "engine-manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
        # Create exclusively: never silently replace a previously accepted artifact.
        with zipfile.ZipFile(output, "x", compression=zipfile.ZIP_DEFLATED) as archive:
            for path in sorted(stage.rglob("*")):
                if path.is_file():
                    archive.write(path, path.relative_to(stage).as_posix())
    print(f"Created {output}")
    print(f"SHA256 {hashlib.sha256(output.read_bytes()).hexdigest()}")


if __name__ == "__main__":
    main()
