"""Turn a recorded episode into events, a summary and a plot."""
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from slipsim.recorder import is_slipping

GROSS_SPEED_MPS = 5e-3      # object speed relative to the palm
GROSS_SUSTAIN_S = 0.01      # must stay above it this long, to ignore spikes
HELD_TOLERANCE_M = 5e-3


def margin_w(fn, ft, mu):
    """Slip margin w = 1 - Ft/(mu*Fn) over all taxels: 1 is firm, 0 is sliding."""
    total_fn = fn.sum(axis=1)
    return 1.0 - ft.sum(axis=1) / np.maximum(mu * total_fn, 1e-9)


def analyse(rec, scenario, watch_start, baseline_rel_z):
    count = rec.frame
    t = rec.t[:count]
    fn, ft, speed = rec.fn[:count], rec.ft[:count], rec.slip_speed[:count]
    slipping = is_slipping(fn, ft, speed, rec.mu)
    w = margin_w(fn, ft, rec.mu)
    rel_z = rec.object_pos[:count, 2] - rec.palm_z[:count]
    baseline = rel_z[0] if baseline_rel_z is None else baseline_rel_z

    summary = {
        "held": bool(abs(rel_z[-1] - baseline) < HELD_TOLERANCE_M),
        "object_rise_mm": float((rec.object_pos[count - 1, 2] - rec.object_pos[0, 2]) * 1e3),
        "relative_shift_mm": float((rel_z[-1] - baseline) * 1e3),
        "touching_taxels": int((fn[-1] > 1e-6).sum()),
        "total_taxels": int(fn.shape[1]),
        "watch_start_s": watch_start,
        "first_taxel_slip_s": None,
        "gross_slip_s": None,
        "lead_s": None,
        "w_at_first_taxel_slip": None,
        "w_at_gross_slip": None,
    }
    if watch_start is None:
        return summary

    watched = t >= watch_start
    first = np.flatnonzero(watched & slipping.any(axis=1))
    if first.size:
        summary["first_taxel_slip_s"] = float(t[first[0]])
        summary["w_at_first_taxel_slip"] = float(w[first[0]])

    frame_dt = t[1] - t[0] if count > 1 else 0.001
    sustain = max(1, int(round(GROSS_SUSTAIN_S / frame_dt)))
    fast = watched & (rec.object_rel_speed[:count] > GROSS_SPEED_MPS)
    for i in np.flatnonzero(fast):
        if fast[i:i + sustain].all() and i + sustain <= count:
            summary["gross_slip_s"] = float(t[i])
            summary["w_at_gross_slip"] = float(w[i])
            break
    if summary["first_taxel_slip_s"] is not None and summary["gross_slip_s"] is not None:
        summary["lead_s"] = summary["gross_slip_s"] - summary["first_taxel_slip_s"]
    return summary


def plot_episode(rec, summary, scenario, path):
    count = rec.frame
    t = rec.t[:count]
    fn, ft = rec.fn[:count], rec.ft[:count]
    slipping = is_slipping(fn, ft, rec.slip_speed[:count], rec.mu)
    touching = np.maximum((fn > 1e-6).sum(axis=1), 1)

    fig, axes = plt.subplots(4, 1, figsize=(9, 9), sharex=True)
    axes[0].plot(t, rec.object_pos[:count, 2] * 1e3, label="object")
    axes[0].plot(t, (rec.palm_z[:count] - rec.palm_z[0]) * 1e3 + rec.object_pos[0, 2] * 1e3,
                 "--", label="hand (same start)")
    axes[0].set_ylabel("height [mm]")
    axes[0].legend(loc="best")
    axes[1].plot(t, fn.sum(axis=1), label="normal, all taxels")
    axes[1].plot(t, ft.sum(axis=1), label="shear, all taxels")
    axes[1].set_ylabel("contact force [N]")
    axes[1].legend(loc="best")
    axes[2].plot(t, np.clip(margin_w(fn, ft, rec.mu), 0, 1))
    axes[2].set_ylabel("slip margin w")
    axes[3].plot(t, 100 * slipping.sum(axis=1) / touching)
    axes[3].set_ylabel("taxels slipping [%]")
    axes[3].set_xlabel("time [s]")
    for ax in axes:
        ax.grid(alpha=0.3)
        if summary["first_taxel_slip_s"] is not None:
            ax.axvline(summary["first_taxel_slip_s"], color="tab:orange", ls=":")
        if summary["gross_slip_s"] is not None:
            ax.axvline(summary["gross_slip_s"], color="tab:red", ls=":")
    hand = scenario.hand
    hand_text = f"{hand.n_fingers} fingers" if hand.type == "radial" else f"{len(hand.pads)} pads"
    axes[0].set_title(f"{scenario.name}: {hand_text}, mu={scenario.object.mu} "
                      f"(orange: first taxel slips, red: object slips)")
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)
