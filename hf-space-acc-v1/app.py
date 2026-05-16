#!/usr/bin/env python3
import os
import re
import shutil
import subprocess
import tempfile
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterator, Optional

import gradio as gr

ROOT_DIR = Path(__file__).resolve().parent
PROJECT_DIR = ROOT_DIR / "project"
JOBS_ROOT = Path("/tmp/monosafe_space_jobs")

ANSI_RE = re.compile(r"\x1b\[[0-9;]*m")


class PipelineError(RuntimeError):
    pass


@dataclass
class JobConfig:
    mode: str
    device_class: str
    device_id: int
    opencl_opts: str
    state_eta: list[float]
    state_lb: list[float]
    state_ub: list[float]
    fps: int
    dpi: int
    sample: int


@dataclass
class JobContext:
    config: JobConfig
    run_dir: Path
    output_video: Optional[Path] = None


class PipelineStage:
    name = "base"

    def stream(self, ctx: JobContext) -> Iterator[str]:
        raise NotImplementedError


class ACCVideoStage(PipelineStage):
    name = "acc_video"

    def stream(self, ctx: JobContext) -> Iterator[str]:
        cfg_src = PROJECT_DIR / "examples" / "acc" / "acc.cfg"
        dyn_src = PROJECT_DIR / "examples" / "acc" / "acc_dynamics.cl"
        cfg_dst = ctx.run_dir / "acc.cfg"
        dyn_dst = ctx.run_dir / "acc_dynamics.cl"

        shutil.copy2(cfg_src, cfg_dst)
        shutil.copy2(dyn_src, dyn_dst)
        write_state_grid_overrides(cfg_dst, ctx.config.state_eta, ctx.config.state_lb, ctx.config.state_ub)
        yield (
            f"[stage:{self.name}] Grid overrides: "
            f"eta={format_float_list(ctx.config.state_eta)} "
            f"lb={format_float_list(ctx.config.state_lb)} "
            f"ub={format_float_list(ctx.config.state_ub)}"
        )

        output_video = ctx.run_dir / f"acc_{ctx.config.mode}_evolution.mp4"

        cmd = [
            str(PROJECT_DIR / "run_threshold_synthesis.sh"),
            "--cfg",
            str(cfg_dst),
            "--mode",
            ctx.config.mode,
            "--device-class",
            ctx.config.device_class,
            "--device",
            str(ctx.config.device_id),
            "--output",
            str(output_video),
            "--fps",
            str(ctx.config.fps),
            "--dpi",
            str(ctx.config.dpi),
            "--sample",
            str(ctx.config.sample),
        ]
        if ctx.config.opencl_opts:
            cmd.extend(["--opencl-opts", ctx.config.opencl_opts])

        env = os.environ.copy()
        env["PATH"] = f"/opt/pfaces/bin:{env.get('PATH', '')}"
        env["PFACES_SDK_ROOT"] = "/opt/pfaces/pfaces-sdk"

        yield f"[stage:{self.name}] Running: {' '.join(cmd)}"
        process = subprocess.Popen(
            cmd,
            cwd=str(PROJECT_DIR),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            env=env,
        )

        assert process.stdout is not None
        for line in process.stdout:
            yield line.rstrip("\n")

        rc = process.wait()
        if rc != 0:
            raise PipelineError(f"ACC synthesis/video command failed with exit code {rc}.")
        if not output_video.exists():
            raise PipelineError(f"Expected output video was not produced: {output_video}")
        if output_video.stat().st_size == 0:
            raise PipelineError(f"Output video is empty: {output_video}")

        ctx.output_video = output_video
        yield f"[stage:{self.name}] Output: {output_video} ({output_video.stat().st_size} bytes)"


def strip_ansi(text: str) -> str:
    return ANSI_RE.sub("", text)


def run_capture(cmd: list[str]) -> tuple[int, str]:
    proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, check=False)
    return proc.returncode, strip_ansi(proc.stdout)


def parse_float_csv(text: str, field_name: str) -> list[float]:
    raw = (text or "").strip()
    if not raw:
        raise PipelineError(f"{field_name} cannot be empty.")
    parts = [p.strip() for p in raw.split(",")]
    if any(p == "" for p in parts):
        raise PipelineError(f"{field_name} must be a comma-separated numeric list.")
    try:
        return [float(p) for p in parts]
    except ValueError as exc:
        raise PipelineError(f"{field_name} contains non-numeric values: {raw}") from exc


def format_float_list(values: list[float]) -> str:
    return ",".join(f"{v:g}" for v in values)


def replace_quoted_cfg_value(block_text: str, key: str, value: str) -> str:
    pattern = rf'(^\s*{re.escape(key)}\s*=\s*")[^"]*(";\s*(?:#.*)?$)'
    replaced, count = re.subn(pattern, rf'\g<1>{value}\g<2>', block_text, flags=re.MULTILINE)
    if count != 1:
        raise PipelineError(f"Could not find unique '{key}' entry inside states block.")
    return replaced


def read_default_state_grid() -> tuple[int, list[float], list[float], list[float]]:
    cfg_path = PROJECT_DIR / "examples" / "acc" / "acc.cfg"
    text = cfg_path.read_text(encoding="utf-8")
    m = re.search(r"states\s*\{(.*?)\}", text, flags=re.DOTALL)
    if not m:
        raise PipelineError(f"Could not find states block in template cfg: {cfg_path}")
    block = m.group(1)

    dim_m = re.search(r'^\s*dim\s*=\s*"(\d+)"\s*;', block, flags=re.MULTILINE)
    eta_m = re.search(r'^\s*eta\s*=\s*"([^"]+)"\s*;', block, flags=re.MULTILINE)
    lb_m = re.search(r'^\s*lb\s*=\s*"([^"]+)"\s*;', block, flags=re.MULTILINE)
    ub_m = re.search(r'^\s*ub\s*=\s*"([^"]+)"\s*;', block, flags=re.MULTILINE)
    if not dim_m or not eta_m or not lb_m or not ub_m:
        raise PipelineError("Template ACC cfg is missing states dim/eta/lb/ub entries.")

    dim = int(dim_m.group(1))
    eta = parse_float_csv(eta_m.group(1), "Template states.eta")
    lb = parse_float_csv(lb_m.group(1), "Template states.lb")
    ub = parse_float_csv(ub_m.group(1), "Template states.ub")

    if len(eta) != dim or len(lb) != dim or len(ub) != dim:
        raise PipelineError(
            f"Template ACC cfg states vector lengths must equal dim={dim} "
            f"(eta={len(eta)}, lb={len(lb)}, ub={len(ub)})."
        )
    return dim, eta, lb, ub


DEFAULT_STATE_DIM, DEFAULT_STATE_ETA, DEFAULT_STATE_LB, DEFAULT_STATE_UB = read_default_state_grid()


def validate_state_grid(state_eta_text: str, state_lb_text: str, state_ub_text: str) -> tuple[list[float], list[float], list[float]]:
    eta = parse_float_csv(state_eta_text, "states.eta")
    lb = parse_float_csv(state_lb_text, "states.lb")
    ub = parse_float_csv(state_ub_text, "states.ub")

    if len(eta) != DEFAULT_STATE_DIM or len(lb) != DEFAULT_STATE_DIM or len(ub) != DEFAULT_STATE_DIM:
        raise PipelineError(
            f"states.eta/states.lb/states.ub must each have {DEFAULT_STATE_DIM} comma-separated values."
        )
    for i, step in enumerate(eta):
        if step <= 0.0:
            raise PipelineError(f"states.eta[{i}] must be > 0 (got {step}).")
    for i, (lo, hi) in enumerate(zip(lb, ub)):
        if hi <= lo:
            raise PipelineError(f"states.ub[{i}] must be greater than states.lb[{i}] (got {hi} <= {lo}).")
    return eta, lb, ub


def write_state_grid_overrides(cfg_path: Path, eta: list[float], lb: list[float], ub: list[float]) -> None:
    text = cfg_path.read_text(encoding="utf-8")
    m = re.search(r"(states\s*\{)(.*?)(\})", text, flags=re.DOTALL)
    if not m:
        raise PipelineError(f"Could not find states block in cfg: {cfg_path}")

    states_head, states_body, states_tail = m.group(1), m.group(2), m.group(3)
    states_body = replace_quoted_cfg_value(states_body, "eta", format_float_list(eta))
    states_body = replace_quoted_cfg_value(states_body, "lb", format_float_list(lb))
    states_body = replace_quoted_cfg_value(states_body, "ub", format_float_list(ub))
    new_states_block = f"{states_head}{states_body}{states_tail}"
    updated = text[: m.start()] + new_states_block + text[m.end() :]
    cfg_path.write_text(updated, encoding="utf-8")


def parse_all_device_ids(pfaces_list_output: str, class_char: str) -> list[int]:
    class_name = "GPU" if class_char == "G" else "CPU"
    ids = []
    for line in pfaces_list_output.splitlines():
        m = re.search(rf"\[(\d+):\s*{class_name}\]", line)
        if m:
            ids.append(int(m.group(1)))
    return ids


def list_pfaces_devices(class_char: str) -> tuple[int, str, list[int]]:
    rc, out = run_capture(["pfaces", f"-{class_char}", "-l"])
    ids = parse_all_device_ids(out, class_char)
    return rc, out, ids


def parse_license_max_pes(pfaces_help_output: str) -> Optional[int]:
    m = re.search(r"Max PEs\s*:\s*(\d+)", pfaces_help_output)
    return int(m.group(1)) if m else None


def parse_gpu_compute_units(clinfo_output: str) -> Optional[int]:
    in_gpu = False
    for raw_line in clinfo_output.splitlines():
        line = raw_line.strip()
        if re.search(r"Device Type\s+GPU", line):
            in_gpu = True
            continue
        if in_gpu and re.search(r"Device Type\s+", line) and "GPU" not in line:
            in_gpu = False
        if in_gpu and "Max compute units" in line:
            parts = line.split()
            try:
                return int(parts[-1])
            except (ValueError, IndexError):
                return None
    return None


def resolve_device(preferred_device: Optional[int], preferred_class: str) -> tuple[str, int, str, str]:
    if shutil.which("pfaces") is None:
        raise PipelineError("pfaces is not available in PATH.")

    mode = (preferred_class or "auto").strip().lower()
    if mode not in {"auto", "gpu", "cpu"}:
        raise PipelineError("Device class must be one of: auto, gpu, cpu.")

    gpu_rc, gpu_out, gpu_ids = list_pfaces_devices("G")
    cpu_rc, cpu_out, cpu_ids = list_pfaces_devices("C")

    debug = [
        f"pfaces -G -l rc={gpu_rc}; gpu_ids={gpu_ids}",
        f"pfaces -C -l rc={cpu_rc}; cpu_ids={cpu_ids}",
    ]

    if mode == "gpu":
        if not gpu_ids:
            raise PipelineError(
                "GPU mode requested but no pFaces OpenCL GPU was detected.\n"
                + "\n".join(debug)
            )
        selected_class = "G"
        available_ids = gpu_ids
    elif mode == "cpu":
        if not cpu_ids:
            raise PipelineError(
                "CPU mode requested but no pFaces OpenCL CPU device was detected.\n"
                + "\n".join(debug)
            )
        selected_class = "C"
        available_ids = cpu_ids
    else:
        if gpu_ids:
            selected_class = "G"
            available_ids = gpu_ids
        elif cpu_ids:
            selected_class = "C"
            available_ids = cpu_ids
        else:
            raise PipelineError(
                "No suitable pFaces OpenCL devices were detected (GPU or CPU).\n"
                + "\n".join(debug)
            )

    if preferred_device is not None:
        if preferred_device not in available_ids:
            class_name = "GPU" if selected_class == "G" else "CPU"
            raise PipelineError(
                f"Requested device id {preferred_device} is not in available {class_name} devices: {available_ids}."
            )
        selected_device = preferred_device
    else:
        selected_device = available_ids[0]

    # Linux pFaces build defaults to CL2.0, while common CPU ICDs expose CL1.2.
    opencl_opts = "-cl-std=CL1.2" if selected_class == "C" else ""

    details = [
        f"selected_class={selected_class}",
        f"selected_device={selected_device}",
        f"opencl_opts={opencl_opts or '(none)'}",
    ]

    if selected_class == "G":
        _, pfaces_help = run_capture(["pfaces", "-h"])
        license_max_pes = parse_license_max_pes(pfaces_help)
        gpu_compute_units = None
        if shutil.which("clinfo") is not None:
            _, clinfo_out = run_capture(["clinfo"])
            gpu_compute_units = parse_gpu_compute_units(clinfo_out)
        if license_max_pes is not None and gpu_compute_units is not None and gpu_compute_units > license_max_pes:
            raise PipelineError(
                "pFaces demo license limit exceeded: "
                f"license max PEs={license_max_pes}, GPU compute units={gpu_compute_units}."
            )

    return selected_class, selected_device, "\n".join(details), opencl_opts


def build_pipeline() -> list[PipelineStage]:
    return [ACCVideoStage()]


def parse_device_id(device_text: str) -> Optional[int]:
    device_text = (device_text or "").strip()
    if not device_text:
        return None
    try:
        return int(device_text)
    except ValueError as exc:
        raise PipelineError("Device id must be an integer.") from exc


def startup_diagnostics() -> str:
    lines = []
    lines.append("### Runtime Diagnostics")
    lines.append(f"- Timestamp (UTC): {datetime.utcnow().isoformat(timespec='seconds')}Z")

    pfaces_path = shutil.which("pfaces")
    lines.append(f"- pfaces path: `{pfaces_path or 'not found'}`")

    if pfaces_path is None:
        return "\n".join(lines)

    rc, help_out = run_capture(["pfaces", "-h"])
    lines.append(f"- pfaces -h rc: {rc}")
    if rc == 0:
        first_line = help_out.splitlines()[0] if help_out.splitlines() else "(no output)"
        lines.append(f"- pfaces version: `{first_line}`")

    gpu_rc, gpu_out, gpu_ids = list_pfaces_devices("G")
    cpu_rc, cpu_out, cpu_ids = list_pfaces_devices("C")
    lines.append(f"- pfaces -G -l rc: {gpu_rc}, gpu_ids: {gpu_ids}")
    lines.append(f"- pfaces -C -l rc: {cpu_rc}, cpu_ids: {cpu_ids}")

    if shutil.which("clinfo") is not None:
        cl_rc, cl_out = run_capture(["clinfo"])
        lines.append(f"- clinfo rc: {cl_rc}")
        preview = "\\n".join(cl_out.splitlines()[:18])
        lines.append("```text")
        lines.append(preview)
        lines.append("```")

    try:
        selected_class, selected_device, details, opencl_opts = resolve_device(None, "auto")
        lines.append(
            "- Auto-selection: "
            f"class `{selected_class}`, device `{selected_device}`, opencl_opts `{opencl_opts or '(none)'}`"
        )
        lines.append("```text")
        lines.append(details)
        lines.append("```")
    except Exception as exc:
        lines.append(f"- Auto-selection failed: `{exc}`")

    # Include condensed raw lists to simplify support debugging.
    lines.append("```text")
    lines.append("pfaces -G -l output:")
    lines.extend(gpu_out.splitlines()[:40])
    lines.append("")
    lines.append("pfaces -C -l output:")
    lines.extend(cpu_out.splitlines()[:40])
    lines.append("```")

    return "\n".join(lines)


def run_job(
    mode: str,
    device_class: str,
    device_text: str,
    state_eta_text: str,
    state_lb_text: str,
    state_ub_text: str,
    fps: int,
    dpi: int,
    sample: int,
):
    logs: list[str] = []

    def emit(status: str, message: str, video_path: Optional[str]):
        logs.append(message)
        return status, "\n".join(logs), video_path

    try:
        yield emit("Preflight", "Starting preflight checks...", None)
        JOBS_ROOT.mkdir(parents=True, exist_ok=True)

        state_eta, state_lb, state_ub = validate_state_grid(state_eta_text, state_lb_text, state_ub_text)
        yield emit(
            "Preflight",
            "Grid validation passed. "
            f"eta={format_float_list(state_eta)} "
            f"lb={format_float_list(state_lb)} "
            f"ub={format_float_list(state_ub)}",
            None,
        )

        device_id = parse_device_id(device_text)
        selected_class, selected_device, selection_details, opencl_opts = resolve_device(device_id, device_class)
        yield emit(
            "Preflight",
            "Device selection passed. "
            f"class={selected_class}, device={selected_device}, opencl_opts={opencl_opts or '(none)'}",
            None,
        )
        yield emit("Preflight", selection_details, None)

        run_dir = Path(tempfile.mkdtemp(prefix="acc_job_", dir=str(JOBS_ROOT)))
        cfg = JobConfig(
            mode=mode,
            device_class=selected_class,
            device_id=selected_device,
            opencl_opts=opencl_opts,
            state_eta=state_eta,
            state_lb=state_lb,
            state_ub=state_ub,
            fps=int(fps),
            dpi=int(dpi),
            sample=int(sample),
        )
        ctx = JobContext(config=cfg, run_dir=run_dir)

        stages = build_pipeline()
        for stage in stages:
            yield emit("Running", f"Entering stage: {stage.name}", None)
            for line in stage.stream(ctx):
                yield emit("Running", line, None)

        if ctx.output_video is None:
            raise PipelineError("Pipeline completed without producing a video path.")

        yield emit("Completed", f"Job complete. Video: {ctx.output_video}", str(ctx.output_video))

    except Exception as exc:
        yield emit("Failed", f"ERROR: {exc}", None)


def build_ui() -> gr.Blocks:
    JOBS_ROOT.mkdir(parents=True, exist_ok=True)

    with gr.Blocks(title="MonoSafe ACC Video Generator") as demo:
        gr.Markdown("# MonoSafe ACC Video Generator (pFaces 1.4)")
        gr.Markdown(
            "This Space runs ACC threshold/basis synthesis in a queued background job and returns the generated video. "
            "Device mode defaults to auto (GPU first, CPU fallback)."
        )
        gr.Markdown(startup_diagnostics())

        with gr.Row():
            mode = gr.Dropdown(
                label="ACC Mode",
                choices=["threshold", "basis"],
                value="threshold",
            )
            device_class = gr.Dropdown(
                label="Device Class",
                choices=["auto", "gpu", "cpu"],
                value="auto",
            )
            device_id = gr.Textbox(
                label="pFaces Device ID (optional)",
                value="",
                placeholder="Leave empty for auto",
            )

        with gr.Row():
            fps = gr.Slider(label="FPS", minimum=5, maximum=30, step=1, value=15)
            dpi = gr.Slider(label="DPI", minimum=80, maximum=220, step=10, value=120)
            sample = gr.Slider(label="Sample Every N Iterations", minimum=1, maximum=10, step=1, value=2)

        with gr.Accordion("Grid Specification (states)", open=False):
            state_eta = gr.Textbox(
                label="states.eta (resolution)",
                value=format_float_list(DEFAULT_STATE_ETA),
                placeholder="e.g., 0.8,0.4,0.4",
            )
            state_lb = gr.Textbox(
                label="states.lb (lower bounds)",
                value=format_float_list(DEFAULT_STATE_LB),
                placeholder="e.g., 0.0,0.0,0.0",
            )
            state_ub = gr.Textbox(
                label="states.ub (upper bounds)",
                value=format_float_list(DEFAULT_STATE_UB),
                placeholder="e.g., 80.0,20.0,20.0",
            )

        run_btn = gr.Button("Generate ACC Video", variant="primary")

        status = gr.Textbox(label="Status", interactive=False)
        logs = gr.Textbox(label="Run Log", lines=22, interactive=False)
        video = gr.Video(label="Generated Video")

        run_btn.click(
            fn=run_job,
            inputs=[mode, device_class, device_id, state_eta, state_lb, state_ub, fps, dpi, sample],
            outputs=[status, logs, video],
        )

    return demo


if __name__ == "__main__":
    ui = build_ui()
    ui.queue(default_concurrency_limit=1)
    ui.launch(server_name="0.0.0.0", server_port=7860)
