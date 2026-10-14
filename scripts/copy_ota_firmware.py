Import("env", "projenv")

import re
import shutil
import subprocess
from pathlib import Path

# Tag v1.2.3 or 1.2.3. The copied file is always waterwell-ota-v1.2.3.bin.
VERSION_TAG = re.compile(r"^[vV]?(\d+\.\d+\.\d+)$")


def version_from_tag(tag):
    match = VERSION_TAG.match(tag.strip())
    if not match:
        return None
    version = match.group(1)
    numbers = tuple(int(part) for part in version.split("."))
    return numbers, version


def git_output(project_dir, args):
    try:
        return subprocess.check_output(
            ["git", *args],
            cwd=str(project_dir),
            text=True,
            stderr=subprocess.STDOUT,
        )
    except FileNotFoundError as exc:
        raise RuntimeError("git was not found, so the firmware version cannot be read.") from exc
    except subprocess.CalledProcessError as exc:
        raise RuntimeError(
            "No git tag found. Create a version tag such as v0.1.0 before building."
        ) from exc


def git_version(project_dir):
    # Prefer a version tag on this commit. Several tags: take the highest.
    pointed = []
    for line in git_output(project_dir, ["tag", "--points-at", "HEAD"]).splitlines():
        parsed = version_from_tag(line)
        if parsed is not None:
            pointed.append(parsed)
    if pointed:
        return max(pointed)[1]

    tag = git_output(project_dir, ["describe", "--tags", "--abbrev=0"]).strip()
    parsed = version_from_tag(tag)
    if parsed is None:
        raise RuntimeError(
            "Git tag '%s' is not a version. Use vX.Y.Z, for example v0.1.0." % tag
        )
    return parsed[1]


def copy_firmware(source, target, env):
    project_dir = Path(env.subst("$PROJECT_DIR"))
    try:
        version = git_version(project_dir)
    except RuntimeError as exc:
        print("Error: %s" % exc)
        env.Exit(1)

    dest_dir = project_dir / "update"
    dest_dir.mkdir(parents=True, exist_ok=True)
    # Keep the original name for the WROOM build. Other envs get a suffix so
    # an ESP32-S3 image does not replace the classic ESP32 OTA file.
    env_name = env["PIOENV"]
    suffix = "" if env_name == "esp-wroom-32" else ("-" + env_name)
    dest = dest_dir / ("waterwell-ota-v%s%s.bin" % (version, suffix))
    shutil.copy2(target[0].get_abspath(), dest)
    print("Copied firmware to %s" % dest)


def define_firmware_version():
    # The version ends up in the binary as "vX.Y.Z" (FW_VERSION). A missing tag
    # must not break the compile here: copy_firmware still refuses to copy.
    try:
        version = git_version(Path(env.subst("$PROJECT_DIR")))
    except RuntimeError as exc:
        print("Warning: %s FW_VERSION will be 'unknown'." % exc)
        return
    # Sources under src/ are compiled with projenv, not env.
    projenv.Append(CPPDEFINES=[("FW_VERSION", env.StringifyMacro("v" + version))])


define_firmware_version()
env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", copy_firmware)
