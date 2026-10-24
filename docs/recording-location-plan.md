# App-owned MCAP output location

Status: Implemented, verification complete. Implementation commit: Pending.

Goal: Configure future MCAP locations in bullet_app/configs/app.json, with a
per-launch CLI override. The launcher continues to supervise the recorder;
standalone bullet-app behavior and all process-log locations remain unchanged.

Implement a focused strict JSON loader. Require recording.output_dir as a
nonempty string; reject missing/unknown fields and malformed JSON. Resolve file
paths beside the configuration file, CLI paths from the working directory, and
expand ~. Default uses the workspace recordings directory, updated by the user after implementation. The launcher supports
--app-config and --mcap-output-dir, and passes --output-dir to mcap-bagger.
Recorder.init gains a keyword-only output_dir override, retaining the recorder
YAML's original base for transport/schema paths. No config file rewriting.

Before readiness, create the directory and write/clean a temporary probe. Errors
name the directory and abort launch before Godot/physics/app start. Print the
resolved MCAP directory. Keep filenames, one-file-per-flight, partial files and
shutdown unchanged; do not move existing recordings or add runtime switching.

Verify defaults, relative/absolute/~/override paths, different CWDs, strict input
validation, path failure and simulated permission/write errors, readiness order,
unchanged logs and YAML-relative dependencies. Record two short TCP-controlled
episodes into a custom folder and independently read the MCAPs. Run affected
app/recorder/launcher regressions. No new dependencies, ports, commits or pushes.


Verification outcomes:
- All 105 app tests passed, including 21 configuration/path validation cases.
- All 85 recorder tests passed, including directory override, unchanged YAML
  transport/schema bases, clean probes, invalid targets, simulated permission/
  write failures and readiness ordering.
- All 3 launcher tests passed, verifying CLI override forwarding, unchanged log
  paths and shutdown order, plus no flight startup after config/recorder failure.
- A real recorder subprocess recorded two TCP-controlled episodes with SHM images
  into a custom nested directory. Independent MCAP readback confirmed distinct
  filenames, three correct images per episode, complete health and episode IDs.
  The original configured destination was not created; no probe/partial files
  remained after successful finalization.
- Shared uv lock consistency and changed-file whitespace checks passed. No new
  dependencies or protocol changes. Standalone bullet-app remains unchanged.

Limit: no live flight was run; the recording/location path was verified with
real recorder processes, TCP control and SHM images. Changes are uncommitted.
