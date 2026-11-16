"""
mal_log_plot.py  —  Plot MAL_log XML measurements with their constraint limits.

Usage
-----
    python utils/mal_log_plot.py path/to/file1.xml [file2.xml ...] [--save] [--out-dir DIR]
                                 [--zones ZONE ...] [--no-joint-limits] [--plots PLOT ...] [--stitch]
                                 [--xml-dir DIR]

Available plot keys (--plots):
  tcp_ft        TCP force/torque    (cartesian component limits per zone)
  tcp_pose      TCP pose            (position + quaternion, no constraints)
  tcp_vel       TCP velocity        (cartesian velocity limits per zone)
  tcp_acc       TCP acceleration    (cartesian acceleration limits per zone)
  joint_torque  Joint torques       (joint-level torque limits)
  joint_angles  Joint angles        (rad → deg, no constraints)
  joint_ang_vel Joint ang. velocity (angle_velocity limits)
  joint_ang_acc Joint ang. accel.   (angle_acceleration limits)
  joint_temp    Joint temperature   (temperature limits)
  joint_power   Joint power         (power limits)
  norms         Norms of velocity, acceleration, joint torque and TCP force/torque
                (cartesian norm limits per zone, joint torque_norm limit)

All available plots are produced by default; use --plots to show a subset.
"""

import argparse
import os
import re
from datetime import datetime

import matplotlib.pyplot as plt
import numpy as np
import xml.etree.ElementTree as ET


# ---------------------------------------------------------------------------
# XML parsing
# ---------------------------------------------------------------------------

def _parse_data_block(text: str, dim: int) -> np.ndarray:
    """Parse newline-separated, comma-delimited rows into an (N, dim) array."""
    rows = []
    for line in text.strip().splitlines():
        line = line.strip()
        if not line:
            continue
        values = [float(v) for v in line.replace(",", " ").split()]
        if len(values) == dim:
            rows.append(values)
    return np.array(rows) if rows else np.empty((0, dim))


def _infer_dim_from_data(text: str) -> int:
    """Infer row dimension from first non-empty data line.

    Handles both comma-separated vectors and scalar-per-line values.
    """
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        return len(line.replace(",", " ").split())
    return 1


def _parse_time_block(text: str) -> np.ndarray:
    """Parse newline-separated timestamps into POSIX seconds.

    The XML writer stores timestamps like '2026-07-17 14:59:27.443'.
    """
    values = []
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            dt = datetime.strptime(line, "%Y-%m-%d %H:%M:%S.%f")
        except ValueError:
            try:
                dt = datetime.strptime(line, "%Y-%m-%d %H:%M:%S")
            except ValueError:
                continue
        values.append(dt.timestamp())
    return np.array(values, dtype=float)


def _parse_scalar_range(element):
    """Return (min_val, max_val) floats from a <...><min/><max/></...> element."""
    if element is None:
        return None, None
    lo = element.find("min")
    hi = element.find("max")
    return (
        float(lo.text) if lo is not None and lo.text else None,
        float(hi.text) if hi is not None and hi.text else None,
    )


def _parse_component_range(element):
    """Return (lo_array, hi_array) from a <component><min/><max/></component> element."""
    if element is None:
        return None, None
    lo = element.find("min")
    hi = element.find("max")
    if lo is None or hi is None:
        return None, None
    return (
        np.array([float(v) for v in lo.text.split(",")]),
        np.array([float(v) for v in hi.text.split(",")]),
    )


def _parse_constraint_block(elem):
    """Parse a <constraints> XML element into a nested dict."""
    c = {}
    for qty in ("force", "torque", "velocity", "acceleration"):
        q = elem.find(qty)
        if q is None:
            continue
        entry = {}
        norm_el = q.find("norm")
        comp_el = q.find("component")
        if norm_el is not None:
            entry["norm"] = _parse_scalar_range(norm_el)
        if comp_el is not None:
            entry["component"] = _parse_component_range(comp_el)
        c[qty] = entry
    for qty in ("angular_velocity", "angular_acceleration"):
        q = elem.find(qty)
        if q is not None:
            c[qty] = _parse_scalar_range(q)
    return c


def _load_mal_roots(xml_path: str) -> list[ET.Element]:
    """Load MAL XML and return all ``<MAL_log>`` root elements in file order.

    Some logs are stored as multiple XML documents concatenated into one file;
    each ``<MAL_log>...</MAL_log>`` block is parsed separately.
    """
    try:
        tree = ET.parse(xml_path)
        return [tree.getroot()]
    except ET.ParseError as exc:
        with open(xml_path, "r", encoding="utf-8", errors="replace") as f:
            xml_text = f.read()

        blocks = re.findall(r"<MAL_log\b[\s\S]*?</MAL_log>", xml_text)
        if not blocks:
            raise exc

        roots = []
        for block in blocks:
            try:
                roots.append(ET.fromstring(block))
            except ET.ParseError:
                continue

        if not roots:
            raise exc

        return roots


def _load_mal_root(xml_path: str) -> ET.Element:
    """Return the last (newest) ``<MAL_log>`` root element in the file."""
    return _load_mal_roots(xml_path)[-1]


def load_mal_logs(xml_path: str) -> list[dict]:
    """Parse every MAL_log entry in an XML file; titles are sequential numbers if >1."""
    roots = _load_mal_roots(xml_path)
    base = os.path.basename(xml_path)
    if len(roots) == 1:
        return [_parse_mal_root(roots[0], base)]
    return [_parse_mal_root(r, str(i + 1)) for i, r in enumerate(roots)]


def load_mal_log(xml_path: str) -> dict:
    """Parse a MAL_log XML file; if it holds several entries, return the first."""
    logs = load_mal_logs(xml_path)
    if len(logs) > 1:
        print(
            f"  [info] Detected {len(logs)} concatenated MAL_log documents; "
            "using the first one (use --stitch to plot all)."
        )
    log = logs[0]
    log["title"] = os.path.basename(xml_path)
    return log


def _parse_mal_root(root: ET.Element, title: str) -> dict:
    """Parse a single ``<MAL_log>`` element into a structured dict."""
    # -- error summary --
    error = {}
    error_el = root.find("Error")
    if error_el is not None:
        error["general"] = error_el.findtext("General", "false").lower() == "true"
        js = error_el.find("Jointspace")
        if js is not None:
            error["joint_ok"] = js.findtext("ok", "true").lower() == "true"
            error["joint_details"] = js.findtext("details", "")
        cs = error_el.find("CartesianSpace")
        if cs is not None:
            error["cart_ok"] = cs.findtext("ok", "true").lower() == "true"
            error["cart_details"] = cs.findtext("details", "")

    # -- constraints --
    constraints = {}
    js_el = root.find("constraints/joint_space")
    if js_el is not None:
        constraints["joint_power"] = _parse_scalar_range(js_el.find("power"))
        # torque_norm may carry an optional noise value
        tn_el = js_el.find("torque_norm")
        constraints["joint_torque_norm"] = _parse_scalar_range(tn_el)
        noise_el = tn_el.find("noise") if tn_el is not None else None
        if noise_el is not None and noise_el.text:
            constraints["joint_torque_norm_noise"] = float(noise_el.text)
        jl = js_el.find("joint_level")
        if jl is not None:
            constraints["joint_torque"] = _parse_scalar_range(jl.find("torque"))
            constraints["joint_angle_velocity"] = _parse_scalar_range(
                jl.find("angle_velocity")
            )
            constraints["joint_angle_acceleration"] = _parse_scalar_range(
                jl.find("angle_acceleration")
            )
            constraints["joint_temperature"] = _parse_scalar_range(
                jl.find("temperature")
            )

    cc_el = root.find("constraints/cartesian_constraints")
    if cc_el is not None:
        for zone in ("free", "expected", "contact", "past_con"):
            zone_el = cc_el.find(zone)
            if zone_el is not None:
                c_el = zone_el.find("constraints")
                if c_el is not None:
                    constraints[f"cart_{zone}"] = _parse_constraint_block(c_el)

    # -- measurements: parse every <key><dim/><data/></key> block generically --
    measurements = {}
    meas_el = root.find("measurements")
    if meas_el is not None:
        for el in meas_el:
            data_text = el.findtext("data", "")
            if not data_text.strip():
                continue
            if el.tag == "time":
                measurements[el.tag] = _parse_time_block(data_text)
                continue
            dim_text = el.findtext("dim")
            dim = int(dim_text) if dim_text is not None else _infer_dim_from_data(data_text)
            measurements[el.tag] = _parse_data_block(data_text, dim)

    return {
        "title": title,
        "timestamp": root.findtext("time_stamp", ""),
        "error": error,
        "constraints": constraints,
        "measurements": measurements,
    }


# ---------------------------------------------------------------------------
# Plotting utilities
# ---------------------------------------------------------------------------

# Colours assigned to each cartesian zone
_ZONE_COLORS = {
    "free": "steelblue",
    "expected": "forestgreen",
    "contact": "darkorange",
    "past_con": "mediumpurple",
}

_TCP_FT_LABELS    = ["Fx [N]", "Fy [N]", "Fz [N]", "Tx [Nm]", "Ty [Nm]", "Tz [Nm]"]
_TCP_POSE_LABELS  = ["x [m]", "y [m]", "z [m]", "qw", "qx", "qy", "qz"]
_TCP_VEL_LABELS   = ["vx [m/s]", "vy [m/s]", "vz [m/s]", "ωw", "ωx", "ωy", "ωz"]
_TCP_ACC_LABELS   = ["ax [m/s²]", "ay [m/s²]", "az [m/s²]", "αw", "αx", "αy", "αz"]
_JOINT_LABELS     = [f"J{i + 1}" for i in range(8)]

_ZONES = ("free", "expected", "contact", "past_con")

# Map measurement key → which cartesian constraint quantity and channel split
# (qty, n_translational_channels) — translational channels map to component[0..n-1],
# remaining channels are rotational.
_CART_QTY_MAP = {
    "tcp_ft":  ("force",        3, "torque"),   # first 3 → force, next 3 → torque
    "tcp_vel": ("velocity",     3, None),
    "tcp_acc": ("acceleration", 3, None),
}

# Map measurement key → (constraint_key, ylabel, color_code)
_JOINT_PLOT_MAP = {
    "joint_torque":   ("joint_torque",            "[Nm]",     "b"),
    "joint_ang_vel":  ("joint_angle_velocity",     "[rad/s]",  "g"),
    "joint_ang_acc":  ("joint_angle_acceleration", "[rad/s²]", "c"),
    "joint_temp":     ("joint_temperature",        "[°C]",     "r"),
    "joint_power":    ("joint_power",              "[W]",      "m"),
    "joint_angles":   (None,                       "[deg]",    "purple"),
}


def _add_limit_lines(ax, lo, hi, color="red", label_prefix=""):
    """Draw dashed limit lines and optionally label them."""
    if lo is not None:
        ax.axhline(lo, color=color, linestyle="--", linewidth=1.2,
                   label=f"{label_prefix} min" if label_prefix else None)
    if hi is not None:
        ax.axhline(hi, color=color, linestyle="-.", linewidth=1.2,
                   label=f"{label_prefix} max" if label_prefix else None)


def _get_x_axis(log: dict, n_samples: int) -> tuple[np.ndarray, str]:
    """Return (x_values, xlabel) for plots.

    If the log contains ``measurements/time`` (or legacy ``timestamps``)
    with one value per sample, values are converted to relative seconds and
    used as the x-axis. Otherwise sample indices are used.
    """
    ts = log["measurements"].get("time")
    if ts is None:
        ts = log["measurements"].get("timestamps")

    if ts is not None:
        if ts.ndim == 1:
            t = ts.astype(float)
        elif ts.ndim == 2 and ts.shape[1] == 1:
            t = ts[:, 0].astype(float)
        else:
            t = None

        if t is not None and len(t) == n_samples:
            return t - t[0], "Time [s]"

    return np.arange(n_samples), "Measurement index"


def _get_abs_time_axis(log: dict, n_samples: int) -> np.ndarray | None:
    """Return absolute POSIX time axis if present and aligned with samples."""
    ts = log["measurements"].get("time")
    if ts is None:
        ts = log["measurements"].get("timestamps")

    if ts is None:
        return None

    if ts.ndim == 1:
        t = ts.astype(float)
    elif ts.ndim == 2 and ts.shape[1] == 1:
        t = ts[:, 0].astype(float)
    else:
        return None

    if len(t) != n_samples:
        return None

    return t


def _build_stitched_series(logs: list[dict], meas_key: str):
    """Concatenate one measurement key across logs into a wall-clock timeline.

    Returns
    -------
    (data, x, segments, xlabel) or (None, None, [], xlabel) if unavailable.
    segments: list of (x_start, x_end, file_label)

    Uses absolute wall-clock timestamps and trims overlap from later files.
    """
    parts = []
    x_parts = []
    segments = []
    expected_dim = None
    prev_end = None
    base_time = None

    for log in logs:
        data = log["measurements"].get(meas_key)
        if data is None or data.size == 0:
            continue
        if data.ndim != 2:
            continue

        n_samples, n_dim = data.shape
        if expected_dim is None:
            expected_dim = n_dim
        elif n_dim != expected_dim:
            print(
                f"  [warn] skip {log['title']} for {meas_key}: "
                f"dimension {n_dim} != expected {expected_dim}"
            )
            continue

        x_abs = _get_abs_time_axis(log, n_samples)
        if x_abs is None:
            print(
                f"  [warn] skip {log['title']} for {meas_key}: "
                "missing/alignment-invalid absolute time axis"
            )
            continue

        if np.any(np.diff(x_abs) < 0):
            order = np.argsort(x_abs)
            x_abs = x_abs[order]
            data = data[order]

        keep_mask = np.isfinite(x_abs)
        if prev_end is not None:
            keep_mask &= x_abs >= prev_end

        if not np.any(keep_mask):
            print(
                f"  [info] skip {log['title']} for {meas_key}: "
                "fully overlapped by previous files"
            )
            continue

        data_keep = data[keep_mask]
        x_keep = x_abs[keep_mask]

        if base_time is None:
            base_time = float(x_keep[0])

        x_seg = x_keep - base_time
        prev_end = float(x_keep[-1])

        parts.append(data_keep)
        x_parts.append(x_seg)
        segments.append((float(x_seg[0]), float(x_seg[-1]), log["title"]))

    if not parts:
        xlabel = "Wall-clock since first file start [s]"
        return None, None, [], xlabel

    xlabel = "Wall-clock since first file start [s]"
    return np.vstack(parts), np.concatenate(x_parts), segments, xlabel


def _collect_input_files(xml_files: list[str], xml_dir: str | None) -> list[str]:
    """Collect XML paths from explicit list and optional directory.

    Explicit file paths keep their provided order. Directory files are appended
    in sorted order. Duplicates are removed while preserving first occurrence.
    """
    candidates = list(xml_files)

    if xml_dir:
        if not os.path.isdir(xml_dir):
            raise ValueError(f"Not a directory: {xml_dir}")
        dir_files = sorted(
            os.path.join(xml_dir, name)
            for name in os.listdir(xml_dir)
            if os.path.isfile(os.path.join(xml_dir, name)) and name.lower().endswith(".xml")
        )
        candidates.extend(dir_files)

    seen = set()
    unique = []
    for p in candidates:
        ap = os.path.abspath(p)
        if ap in seen:
            continue
        seen.add(ap)
        unique.append(p)
    return unique


def _log_start_time(log: dict) -> float | None:
    """Return earliest absolute timestamp for a log, if available."""
    for key, value in log.get("measurements", {}).items():
        if key == "time":
            ts = value
            break
    else:
        ts = log.get("measurements", {}).get("timestamps")

    if ts is None:
        return None

    if isinstance(ts, np.ndarray):
        if ts.ndim == 1:
            t = ts.astype(float)
        elif ts.ndim == 2 and ts.shape[1] == 1:
            t = ts[:, 0].astype(float)
        else:
            return None
    else:
        return None

    t = t[np.isfinite(t)]
    if t.size == 0:
        return None
    return float(np.min(t))


def _sort_logs_for_stitch(logs: list[dict]) -> list[dict]:
    """Sort logs by earliest wall-clock timestamp; keep stable order otherwise."""
    indexed = list(enumerate(logs))

    def _key(item):
        idx, log = item
        start = _log_start_time(log)
        return (start is None, start if start is not None else float("inf"), idx)

    return [log for _idx, log in sorted(indexed, key=_key)]


def _add_file_segment_overlays(axes, segments):
    """Draw file segment boundaries and labels on stitched timeline plots."""
    if not segments:
        return

    for idx, (x_start, x_end, _label) in enumerate(segments):
        shade = "0.96" if idx % 2 == 0 else "0.92"
        for ax in axes:
            ax.axvspan(x_start, x_end, color=shade, alpha=0.35, zorder=0)
        if idx > 0:
            for ax in axes:
                ax.axvline(x_start, color="0.35", linestyle="--", linewidth=0.8)

    top_ax = axes[0]
    y_min, y_max = top_ax.get_ylim()
    y_text = y_max - 0.06 * (y_max - y_min)
    for x_start, x_end, label in segments:
        top_ax.text(
            0.5 * (x_start + x_end),
            y_text,
            label,
            ha="center",
            va="top",
            fontsize=7,
            bbox={"boxstyle": "round,pad=0.2", "fc": "white", "ec": "0.7", "alpha": 0.7},
        )


def save(fname: str, figure: plt.Figure):
    """Show a figure full-screen, optionally save it, then close."""
    figure.show()
    manager = plt.get_current_fig_manager()
    manager.full_screen_toggle()
    resp = input(f"Save figure as '{fname}.png'? [Y/n]: ")
    if resp.strip().lower() != "n":
        figure.savefig(fname + ".png", format="png", bbox_inches="tight", dpi=150)
        print(f"  Saved → {fname}.png")
    plt.close(figure)


# ---------------------------------------------------------------------------
# Individual plot functions
# ---------------------------------------------------------------------------

def _make_channel_subplots(n_dim: int, fig_height_per_row: float = 2.5):
    """Create a grid of subplots and return (fig, flat_axes)."""
    ncols = 2
    nrows = (n_dim + ncols - 1) // ncols
    fig, axes = plt.subplots(nrows, ncols,
                             figsize=(12, nrows * fig_height_per_row),
                             sharex=True)
    axes = np.array(axes).flatten()
    return fig, axes, nrows, ncols


def _finalize_subplots(axes, n_dim: int, nrows: int, ncols: int, xlabel="Measurement index"):
    """Hide unused axes and add x-label to the bottom row."""
    for i in range(n_dim, len(axes)):
        axes[i].set_visible(False)
    for ax in axes[max(0, (nrows - 1) * ncols):]:
        ax.set_xlabel(xlabel)
    plt.tight_layout(rect=[0, 0, 1, 0.95])


def plot_tcp_ft(
    log: dict,
    zones: list[str] | None = None,
    x_override: np.ndarray | None = None,
    file_segments: list[tuple[float, float, str]] | None = None,
    x_label_override: str | None = None,
) -> plt.Figure | None:
    """TCP force/torque with cartesian component constraint limits per zone.

    Parameters
    ----------
    zones:
        Which cartesian zones to overlay (default: all four).
    """
    data = log["measurements"].get("tcp_ft")
    if data is None or data.size == 0:
        print("  [skip] no tcp_ft data")
        return None

    n_samples, n_dim = data.shape
    x, xlabel = _get_x_axis(log, n_samples)
    if x_override is not None and len(x_override) == n_samples:
        x = x_override
        xlabel = x_label_override or "Stitched timeline"

    fig, axes_arr = plt.subplots(n_dim, 1, figsize=(10, 2.0 * n_dim + 1), sharex=True)
    axes = list(np.array(axes_arr).flatten())

    err = log.get("error", {})
    subtitle = ""
    if not err.get("cart_ok", True):
        subtitle = f"  ⚠ Cart. error: {err.get('cart_details', '')}"
    fig.suptitle(
        f"{log['title']}\nTCP Force / Torque   {log['timestamp']}{subtitle}",
        fontsize=10,
        color="red" if subtitle else "black",
    )

    active_zones = zones if zones is not None else list(_ZONES)

    legend_drawn = False
    for i, ax in enumerate(axes):
        ax.plot(x, data[:, i], "ko-", markersize=4, linewidth=1.5, zorder=3)
        ax.set_ylabel(_TCP_FT_LABELS[i] if i < len(_TCP_FT_LABELS) else f"ch{i}", fontsize=8)
        ax.grid(True, "both")

        qty = "force" if i < 3 else "torque"
        comp_idx = i if i < 3 else i - 3

        for zone in active_zones:
            cz = log["constraints"].get(f"cart_{zone}", {})
            cq = cz.get(qty, {})
            comp_lo, comp_hi = cq.get("component", (None, None))
            color = _ZONE_COLORS.get(zone, "grey")
            lo = comp_lo[comp_idx] if comp_lo is not None and comp_idx < len(comp_lo) else None
            hi = comp_hi[comp_idx] if comp_hi is not None and comp_idx < len(comp_hi) else None
            _add_limit_lines(ax, lo, hi, color=color,
                             label_prefix=zone if not legend_drawn else "")

        if not legend_drawn:
            handles, labels = ax.get_legend_handles_labels()
            if handles:
                ax.legend(fontsize=7, loc="upper right", ncol=2)
                legend_drawn = True

    if file_segments:
        _add_file_segment_overlays(axes, file_segments)

    axes[-1].set_xlabel(xlabel)
    plt.tight_layout(rect=[0, 0, 1, 0.95])
    return fig


def plot_tcp_generic(
    log: dict,
    meas_key: str,
    zones: list[str] | None = None,
    x_override: np.ndarray | None = None,
    file_segments: list[tuple[float, float, str]] | None = None,
    x_label_override: str | None = None,
) -> plt.Figure | None:
    """Generic TCP plot for tcp_pose, tcp_vel, tcp_acc with appropriate constraint overlays."""
    data = log["measurements"].get(meas_key)
    if data is None or data.size == 0:
        print(f"  [skip] no {meas_key} data")
        return None

    n_samples, n_dim = data.shape
    x, xlabel = _get_x_axis(log, n_samples)
    if x_override is not None and len(x_override) == n_samples:
        x = x_override
        xlabel = x_label_override or "Stitched timeline"
    active_zones = zones if zones is not None else list(_ZONES)

    label_map = {
        "tcp_pose": (_TCP_POSE_LABELS, "TCP Pose", "ko-"),
        "tcp_vel":  (_TCP_VEL_LABELS,  "TCP Velocity", "bo-"),
        "tcp_acc":  (_TCP_ACC_LABELS,  "TCP Acceleration", "go-"),
    }
    ch_labels, title_str, marker = label_map.get(meas_key, ([], meas_key, "ko-"))

    fig, axes_arr = plt.subplots(n_dim, 1, figsize=(10, 2.0 * n_dim + 1), sharex=True)
    axes = list(np.array(axes_arr).flatten())
    fig.suptitle(f"{log['title']}\n{title_str}   {log['timestamp']}", fontsize=10)

    # Which cartesian quantity applies? (qty_primary, qty_secondary)
    qty_info = _CART_QTY_MAP.get(meas_key)  # (qty, n_trans, qty2)
    legend_drawn = False

    for i, ax in enumerate(axes):
        ax.plot(x, data[:, i], marker, markersize=4, linewidth=1.5, zorder=3)
        ax.set_ylabel(ch_labels[i] if i < len(ch_labels) else f"ch{i}", fontsize=8)
        ax.grid(True, "both")

        if qty_info is not None:
            qty_primary, n_trans, qty_secondary = qty_info
            if i < n_trans:
                qty = qty_primary
                comp_idx = i
            elif qty_secondary and (n_trans <= i < n_trans + 3):
                qty = qty_secondary
                comp_idx = i - n_trans
            else:
                qty = None

            if qty is not None:
                for zone in active_zones:
                    cz = log["constraints"].get(f"cart_{zone}", {})
                    cq = cz.get(qty, {})
                    comp_lo, comp_hi = cq.get("component", (None, None))
                    color = _ZONE_COLORS.get(zone, "grey")
                    lo = comp_lo[comp_idx] if comp_lo is not None and comp_idx < len(comp_lo) else None
                    hi = comp_hi[comp_idx] if comp_hi is not None and comp_idx < len(comp_hi) else None
                    _add_limit_lines(ax, lo, hi, color=color,
                                     label_prefix=zone if not legend_drawn else "")

        if not legend_drawn:
            handles, labels = ax.get_legend_handles_labels()
            if handles:
                ax.legend(fontsize=7, loc="upper right", ncol=2)
                legend_drawn = True

    if file_segments:
        _add_file_segment_overlays(axes, file_segments)

    axes[-1].set_xlabel(xlabel)
    plt.tight_layout(rect=[0, 0, 1, 0.95])
    return fig


def plot_joint_torques(
    log: dict,
    show_limits: bool = True,
    x_override: np.ndarray | None = None,
    file_segments: list[tuple[float, float, str]] | None = None,
    x_label_override: str | None = None,
) -> plt.Figure | None:
    """Joint torques with joint-level torque limits.

    Parameters
    ----------
    show_limits:
        Whether to draw the min/max limit lines.
    """
    data = log["measurements"].get("joint_torque")
    if data is None or data.size == 0:
        print("  [skip] no joint_torque data")
        return None

    n_samples, n_dim = data.shape
    x, xlabel = _get_x_axis(log, n_samples)
    if x_override is not None and len(x_override) == n_samples:
        x = x_override
        xlabel = x_label_override or "Stitched timeline"
    torque_lo, torque_hi = log["constraints"].get("joint_torque", (None, None))

    fig, axes, nrows, ncols = _make_channel_subplots(n_dim)

    err = log.get("error", {})
    subtitle = (f"  ⚠ Joint error: {err.get('joint_details', '')}"
                if not err.get("joint_ok", True) else "")
    fig.suptitle(
        f"{log['title']}\nJoint Torques   {log['timestamp']}{subtitle}",
        fontsize=10,
        color="red" if subtitle else "black",
    )

    for i in range(n_dim):
        ax = axes[i]
        ax.plot(x, data[:, i], "bo-", markersize=4, linewidth=1.5)
        ax.set_title(_JOINT_LABELS[i] if i < len(_JOINT_LABELS) else f"J{i+1}", fontsize=9)
        ax.set_ylabel("[Nm]", fontsize=8)
        ax.grid(True, "both")
        if show_limits:
            _add_limit_lines(ax, torque_lo, torque_hi, color="red",
                             label_prefix="limit" if i == 0 else "")
        if i == 0:
            handles, _ = ax.get_legend_handles_labels()
            if handles:
                ax.legend(fontsize=7)

    if file_segments:
        _add_file_segment_overlays(axes[:n_dim], file_segments)

    _finalize_subplots(axes, n_dim, nrows, ncols, xlabel)
    return fig


def plot_joint_generic(
    log: dict,
    meas_key: str,
    show_limits: bool = True,
    x_override: np.ndarray | None = None,
    file_segments: list[tuple[float, float, str]] | None = None,
    x_label_override: str | None = None,
) -> plt.Figure | None:
    """Generic joint-space plot for ang_vel, ang_acc, temp, power, and angles.

    Parameters
    ----------
    meas_key:
        One of the keys in ``_JOINT_PLOT_MAP``.
    show_limits:
        Whether to draw the min/max limit lines (ignored for keys without constraints).
    """
    if meas_key not in _JOINT_PLOT_MAP:
        print(f"  [skip] unknown joint plot key '{meas_key}'")
        return None
    data = log["measurements"].get(meas_key)
    if data is None or data.size == 0:
        print(f"  [skip] no {meas_key} data")
        return None

    constraint_key, ylabel, color_code = _JOINT_PLOT_MAP[meas_key]
    n_samples, n_dim = data.shape
    x, xlabel = _get_x_axis(log, n_samples)
    if x_override is not None and len(x_override) == n_samples:
        x = x_override
        xlabel = x_label_override or "Stitched timeline"

    lim_lo, lim_hi = (None, None)
    if constraint_key:
        lim_lo, lim_hi = log["constraints"].get(constraint_key, (None, None))

    title_map = {
        "joint_ang_vel":  "Joint Angular Velocities",
        "joint_ang_acc":  "Joint Angular Accelerations",
        "joint_angles":   "Joint Angles",
        "joint_temp":     "Joint Temperatures",
        "joint_power":    "Joint Power",
    }
    title_str = title_map.get(meas_key, meas_key)

    fig, axes, nrows, ncols = _make_channel_subplots(n_dim)
    fig.suptitle(f"{log['title']}\n{title_str}   {log['timestamp']}", fontsize=10)

    for i in range(n_dim):
        ax = axes[i]
        # Joint angles are stored in radians → convert to degrees
        plot_data = np.degrees(data[:, i]) if meas_key == "joint_angles" else data[:, i]
        ax.plot(x, plot_data, color=color_code, marker="o", markersize=4, linewidth=1.5)
        ax.set_title(_JOINT_LABELS[i] if i < len(_JOINT_LABELS) else f"J{i+1}", fontsize=9)
        ax.set_ylabel(ylabel, fontsize=8)
        ax.grid(True, "both")
        if show_limits and constraint_key:
            _add_limit_lines(ax, lim_lo, lim_hi, color="red",
                             label_prefix="limit" if i == 0 else "")
        if i == 0:
            handles, _ = ax.get_legend_handles_labels()
            if handles:
                ax.legend(fontsize=7)

    if file_segments:
        _add_file_segment_overlays(axes[:n_dim], file_segments)

    _finalize_subplots(axes, n_dim, nrows, ncols, xlabel)
    return fig


def plot_norms(
    log: dict,
    zones: list[str] | None = None,
    show_limits: bool = True,
    x_overrides: dict[str, np.ndarray] | None = None,
    file_segments: list[tuple[float, float, str]] | None = None,
    x_label_override: str | None = None,
) -> plt.Figure | None:
    """Norms of velocity, acceleration, joint torque and TCP force/torque with norm limits.

    Velocity/acceleration use TCP (translational) data when available,
    otherwise the joint-space measurements.
    """
    meas = log["measurements"]
    cons = log["constraints"]
    active_zones = zones if zones is not None else list(_ZONES)

    rows = []  # (meas_key, norm_values, ylabel, cart_qty, joint_limit)

    def _add(key, cols, ylabel, cart_qty=None, joint_lim=None) -> bool:
        data = meas.get(key)
        if data is None or data.ndim != 2 or data.size == 0:
            return False
        sub = data[:, cols]
        if sub.shape[1] == 0:
            return False
        rows.append((key, np.linalg.norm(sub, axis=1), ylabel, cart_qty, joint_lim))
        return True

    if not _add("tcp_vel", slice(0, 3), "|v| [m/s]", cart_qty="velocity"):
        _add("joint_ang_vel", slice(None), "|q̇| [rad/s]")
    if not _add("tcp_acc", slice(0, 3), "|a| [m/s²]", cart_qty="acceleration"):
        _add("joint_ang_acc", slice(None), "|q̈| [rad/s²]")
    _add("joint_torque", slice(None), "|τ| [Nm]", joint_lim=cons.get("joint_torque_norm"))
    _add("tcp_ft", slice(0, 3), "|F| [N]", cart_qty="force")
    _add("tcp_ft", slice(3, 6), "|T| [Nm]", cart_qty="torque")

    if not rows:
        print("  [skip] no data for norm plot")
        return None

    fig, axes_arr = plt.subplots(len(rows), 1, figsize=(10, 2.0 * len(rows) + 1),
                                 sharex=True, squeeze=False)
    axes = list(axes_arr[:, 0])

    err = log.get("error", {})
    notes = []
    if not err.get("joint_ok", True):
        notes.append(f"Joint: {err.get('joint_details', '')}")
    if not err.get("cart_ok", True):
        notes.append(f"Cart.: {err.get('cart_details', '')}")
    subtitle = f"  ⚠ {' | '.join(notes)}" if notes else ""
    fig.suptitle(
        f"{log['title']}\nNorms   {log['timestamp']}{subtitle}",
        fontsize=10,
        color="red" if subtitle else "black",
    )

    xlabel = None
    for ax, (key, vals, ylabel, cart_qty, joint_lim) in zip(axes, rows):
        x, xl = _get_x_axis(log, len(vals))
        if x_overrides and key in x_overrides and len(x_overrides[key]) == len(vals):
            x = x_overrides[key]
            xl = x_label_override or "Stitched timeline"
        xlabel = xlabel or xl

        ax.plot(x, vals, "ko-", markersize=4, linewidth=1.5, zorder=3)
        ax.set_ylabel(ylabel, fontsize=8)
        ax.grid(True, "both")

        # A norm is non-negative, so non-positive min limits are not drawn.
        if cart_qty:
            for zone in active_zones:
                lo, hi = cons.get(f"cart_{zone}", {}).get(cart_qty, {}).get("norm", (None, None))
                _add_limit_lines(ax, lo if lo is not None and lo > 0 else None, hi,
                                 color=_ZONE_COLORS.get(zone, "grey"), label_prefix=zone)
        elif joint_lim is not None and show_limits:
            lo, hi = joint_lim
            _add_limit_lines(ax, lo if lo is not None and lo > 0 else None, hi,
                             color="red", label_prefix="limit")

        if ax.get_legend_handles_labels()[0]:
            ax.legend(fontsize=7, loc="upper right", ncol=2)

    if file_segments:
        _add_file_segment_overlays(axes, file_segments)

    axes[-1].set_xlabel(xlabel)
    plt.tight_layout(rect=[0, 0, 1, 0.95])
    return fig


# Keep backward-compatible aliases
def plot_joint_velocities(log: dict, show_limits: bool = True) -> plt.Figure | None:
    return plot_joint_generic(log, "joint_ang_vel", show_limits)


def plot_joint_angles(log: dict) -> plt.Figure | None:
    return plot_joint_generic(log, "joint_angles", show_limits=False)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Plot MAL_log XML measurements with their constraint limits."
    )
    parser.add_argument("xml_files", nargs="*", help="MAL_log XML file(s) to process")
    parser.add_argument(
        "--save",
        action="store_true",
        help="Prompt to save each figure as PNG after displaying",
    )
    parser.add_argument(
        "--out-dir",
        default=".",
        metavar="DIR",
        help="Output directory for saved PNGs (default: current directory)",
    )
    parser.add_argument(
        "--zones",
        nargs="+",
        choices=["free", "expected", "contact", "past_con"],
        default=["free", "expected", "contact", "past_con"],
        metavar="ZONE",
        help=(
            "Cartesian zones whose limits are overlaid on the TCP F/T plot. "
            "Choices: free, expected, contact, past_con  (default: all four). "
            "Example: --zones expected contact"
        ),
    )
    parser.add_argument(
        "--no-joint-limits",
        action="store_true",
        help="Hide joint torque and velocity limit lines from the joint plots",
    )
    _all_plots = [
        "tcp_ft", "tcp_pose", "tcp_vel", "tcp_acc",
        "joint_torque", "joint_angles", "joint_ang_vel",
        "joint_ang_acc", "joint_temp", "joint_power", "norms",
    ]
    parser.add_argument(
        "--plots",
        nargs="+",
        choices=_all_plots,
        default=None,
        metavar="PLOT",
        help=(
            "Which figures to produce (default: all that have data). "
            f"Choices: {', '.join(_all_plots)}"
        ),
    )
    parser.add_argument(
        "--stitch",
        action="store_true",
        help=(
            "Stitch multiple XML files into one wall-clock timeline per plot and "
            "label timeline segments by filename"
        ),
    )
    parser.add_argument(
        "--xml-dir",
        metavar="DIR",
        default=None,
        help="Include all *.xml files from DIR (sorted) in addition to any explicitly listed files",
    )
    args = parser.parse_args()

    try:
        input_files = _collect_input_files(args.xml_files, args.xml_dir)
    except ValueError as exc:
        parser.error(str(exc))

    if not input_files:
        parser.error("No XML inputs found. Provide xml_files and/or --xml-dir DIR")

    logs = []
    stitch_pool = []
    for xml_path in input_files:
        print(f"\n=== {xml_path} ===")
        if args.stitch:
            entries = load_mal_logs(xml_path)
            if len(entries) > 1:
                print(f"  [info] Detected {len(entries)} MAL_log entries; stitching all.")
                if len(input_files) > 1:
                    base = os.path.basename(xml_path)
                    for e in entries:
                        e["title"] = f"{base} #{e['title']}"
            stitch_pool.extend(entries)
            log = entries[-1]
        else:
            entries = [load_mal_log(xml_path)]
            log = entries[0]
        logs.append(log)

        for entry in entries:
            err = entry["error"]
            prefix = f"  [{entry['title']}] " if len(entries) > 1 else "  "
            print(f"{prefix}Joint ok : {err.get('joint_ok', '?')}   Cart ok : {err.get('cart_ok', '?')}")
            if not err.get("cart_ok", True):
                print(f"{prefix}Cart  → {err.get('cart_details', '')}")
            if not err.get("joint_ok", True):
                print(f"{prefix}Joint → {err.get('joint_details', '')}")

    if not logs:
        return

    show_joint_limits = not args.no_joint_limits
    requested = set(args.plots) if args.plots else None

    def _want(key: str) -> bool:
        return requested is None or key in requested

    if args.stitch:
        if len(stitch_pool) < 2:
            print("  [info] --stitch requested but only one MAL_log provided; plotting normally.")
        else:
            stitch_logs = _sort_logs_for_stitch(stitch_pool)
            print(
                "  [info] Stitch order by start time: "
                + " -> ".join(log.get("title", "?") for log in stitch_logs)
            )

            stitched_log = {
                "title": f"stitched_{len(stitch_logs)}_logs",
                "timestamp": f"{stitch_logs[0].get('timestamp', '')} -> {stitch_logs[-1].get('timestamp', '')}",
                "error": {
                    "joint_ok": all(l.get("error", {}).get("joint_ok", True) for l in stitch_logs),
                    "cart_ok": all(l.get("error", {}).get("cart_ok", True) for l in stitch_logs),
                },
                "constraints": stitch_logs[0].get("constraints", {}),
                "measurements": {},
            }

            figs = {}

            if _want("tcp_ft"):
                data, x, seg, xlab = _build_stitched_series(stitch_logs, "tcp_ft")
                if data is not None:
                    stitched_log["measurements"] = {"tcp_ft": data}
                    figs["tcp_ft"] = plot_tcp_ft(
                        stitched_log,
                        zones=args.zones,
                        x_override=x,
                        file_segments=seg,
                        x_label_override=xlab,
                    )

            for tcp_key in ("tcp_pose", "tcp_vel", "tcp_acc"):
                if _want(tcp_key):
                    data, x, seg, xlab = _build_stitched_series(stitch_logs, tcp_key)
                    if data is not None:
                        stitched_log["measurements"] = {tcp_key: data}
                        figs[tcp_key] = plot_tcp_generic(
                            stitched_log,
                            tcp_key,
                            zones=args.zones,
                            x_override=x,
                            file_segments=seg,
                            x_label_override=xlab,
                        )

            if _want("joint_torque"):
                data, x, seg, xlab = _build_stitched_series(stitch_logs, "joint_torque")
                if data is not None:
                    stitched_log["measurements"] = {"joint_torque": data}
                    figs["joint_torque"] = plot_joint_torques(
                        stitched_log,
                        show_limits=show_joint_limits,
                        x_override=x,
                        file_segments=seg,
                        x_label_override=xlab,
                    )

            for jkey in ("joint_angles", "joint_ang_vel", "joint_ang_acc", "joint_temp", "joint_power"):
                if _want(jkey):
                    data, x, seg, xlab = _build_stitched_series(stitch_logs, jkey)
                    if data is not None:
                        stitched_log["measurements"] = {jkey: data}
                        figs[jkey] = plot_joint_generic(
                            stitched_log,
                            jkey,
                            show_limits=show_joint_limits,
                            x_override=x,
                            file_segments=seg,
                            x_label_override=xlab,
                        )

            if _want("norms"):
                norm_meas, norm_x, norm_seg, norm_xlab = {}, {}, [], None
                for key in ("tcp_vel", "tcp_acc", "joint_ang_vel", "joint_ang_acc",
                            "joint_torque", "tcp_ft"):
                    data, x, seg, xlab = _build_stitched_series(stitch_logs, key)
                    if data is not None:
                        norm_meas[key] = data
                        norm_x[key] = x
                        norm_seg = norm_seg or seg
                        norm_xlab = xlab
                if norm_meas:
                    stitched_log["measurements"] = norm_meas
                    figs["norms"] = plot_norms(
                        stitched_log,
                        zones=args.zones,
                        show_limits=show_joint_limits,
                        x_overrides=norm_x,
                        file_segments=norm_seg,
                        x_label_override=norm_xlab,
                    )

            base = "stitched"
            out_prefix = os.path.join(args.out_dir, base)

            if args.save:
                for suffix, fig in figs.items():
                    if fig is not None:
                        save(f"{out_prefix}_{suffix}", fig)
            else:
                plt.show()

            plt.close("all")
            return

    for xml_path, log in zip(input_files, logs):

        base = os.path.splitext(os.path.basename(xml_path))[0]
        out_prefix = os.path.join(args.out_dir, base)

        figs = {}
        if _want("tcp_ft"):
            figs["tcp_ft"]       = plot_tcp_ft(log, zones=args.zones)
        for tcp_key in ("tcp_pose", "tcp_vel", "tcp_acc"):
            if _want(tcp_key):
                figs[tcp_key]    = plot_tcp_generic(log, tcp_key, zones=args.zones)
        if _want("joint_torque"):
            figs["joint_torque"] = plot_joint_torques(log, show_limits=show_joint_limits)
        for jkey in ("joint_angles", "joint_ang_vel", "joint_ang_acc",
                     "joint_temp", "joint_power"):
            if _want(jkey):
                figs[jkey]       = plot_joint_generic(log, jkey,
                                                      show_limits=show_joint_limits)
        if _want("norms"):
            figs["norms"]        = plot_norms(log, zones=args.zones,
                                              show_limits=show_joint_limits)

        if args.save:
            for suffix, fig in figs.items():
                if fig is not None:
                    save(f"{out_prefix}_{suffix}", fig)
        else:
            plt.show()

    plt.close("all")


if __name__ == "__main__":
    main()
