#!/usr/bin/env python3
"""
Test 125: battery state of charge, 1.13 vs 1.17.

Reads the two flight logs, prints the results table and draws the figures.
See ../test_125.md.

    python3 docs/sees/tests/scripts/125_battery_soc.py <v1.13 log.ulg> <v1.17 log.ulg>
"""
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
from pyulog import ULog

REST = 10 * 60  # s, the rest after disarming
FIGURES = os.path.join(os.path.dirname(__file__), "..", "figures")
COLOURS = {"PX4 1.13": "#2a78d6", "PX4 1.17": "#eb6834"}
ARMED = 2       # vehicle_status.arming_state


def load(path):
    """Battery data from a log, with time in minutes since arming."""
    log = ULog(path, ["battery_status", "vehicle_status"])
    battery = log.get_dataset("battery_status").data
    status = log.get_dataset("vehicle_status").data

    armed = status["timestamp"][status["arming_state"] == ARMED] / 1e6
    arm, disarm = armed[0], armed[-1]
    t = battery["timestamp"] / 1e6

    def at(field, time):
        return battery[field][np.searchsorted(t, time) - 1]

    return {
        "minutes": (t - arm) / 60,
        "disarm": (disarm - arm) / 60,
        "percent": battery["remaining"] * 100,
        "current": battery["current_a"],
        "voltage": battery["voltage_v"],
        "used_ah": (battery["discharged_mah"] - at("discharged_mah", arm)) / 1000,
        "armed_percent": at("remaining", arm - 0.5) * 100,
        "predicted": at("remaining", disarm - 0.5) * 100,
        "observed": at("remaining", disarm + REST) * 100,
        "faults": np.unique(battery["faults"]),
        "warnings": np.unique(battery["warning"]),
    }


def print_results(flights):
    print(f"{'':9} {'Armed':>6} {'Predicted':>10} {'Observed':>9} {'Diff':>5}  Warnings  Faults")
    for name, f in flights.items():
        print(f"{name:9} {f['armed_percent']:6.1f} {f['predicted']:10.1f} {f['observed']:9.1f}"
              f" {f['observed'] - f['predicted']:+5.1f}  {f['warnings']}  {f['faults']}")


def plot_battery_percent(flights):
    fig, ax = plt.subplots(figsize=(10, 5.2))
    for name, f in flights.items():
        colour = COLOURS[name]
        ax.plot(f["minutes"], f["percent"], color=colour, lw=2, label=name)
        ax.axvline(f["disarm"], color=colour, lw=1, ls="--")
        ax.plot([f["disarm"], f["disarm"] + REST / 60], [f["predicted"], f["observed"]],
                "o", ms=8, color=colour, mec="white", mew=2, zorder=5)

    table = [f"{'':9}{'Armed':>8}{'Before disarm':>16}{'After 10 min rest':>20}{'Diff':>8}"]
    for name, f in flights.items():
        table.append(f"{name:9}{f['armed_percent']:7.1f}%{f['predicted']:15.1f}%"
                     f"{f['observed']:19.1f}%{f['observed'] - f['predicted']:+8.1f}")
    ax.text(0.2, 34, "\n".join(table), family="monospace", fontsize=9.5,
            bbox=dict(boxstyle="round,pad=0.5", fc="white", ec="#e4e3df"))

    first_disarm = min(f["disarm"] for f in flights.values())
    ax.axvspan(0, first_disarm, color="#e4e3df", alpha=0.35, lw=0)
    ax.text(0.2, 91.5, "Hover (armed): coulomb counting", color="#52514e")
    ax.text(first_disarm + 0.6, 91.5, "Disarmed, resting: voltage-based", color="#52514e")
    ax.set(xlabel="Minutes since arming", ylabel="Battery remaining (%)", xlim=(-1, 22), ylim=(28, 95))
    ax.set_title("Battery % in the same 10 min hover: 1.13 vs 1.17", loc="left", fontweight="bold")
    ax.grid(color="#e4e3df")
    ax.legend(loc="lower right", frameon=False)
    save(fig, "125_battery_percent.png")


def plot_flight_profile(flights):
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for name, f in flights.items():
        colour = COLOURS[name]
        axes[0].plot(f["minutes"], f["current"], color=colour, lw=1, label=name)
        axes[1].plot(f["minutes"], f["voltage"], color=colour, lw=1.2)
        axes[2].plot(f["minutes"], f["used_ah"], color=colour, lw=2)

    axes[0].set_ylabel("Current (A)")
    axes[1].set_ylabel("Pack voltage (V)")
    axes[2].set(ylabel="Charge used (Ah)", xlabel="Minutes since arming", xlim=(-1, 22))
    axes[0].set_title("Same load on both flights: current, voltage and charge used", loc="left", fontweight="bold")
    axes[0].legend(loc="upper right", frameon=False)
    for ax in axes:
        ax.grid(color="#e4e3df")
    fig.tight_layout()
    save(fig, "125_flight_profile.png")


def save(fig, name):
    path = os.path.join(FIGURES, name)
    fig.savefig(path, dpi=160, bbox_inches="tight")
    print("Wrote", os.path.normpath(path))


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)

    flights = {"PX4 1.13": load(sys.argv[1]), "PX4 1.17": load(sys.argv[2])}
    print_results(flights)
    plot_battery_percent(flights)
    plot_flight_profile(flights)


if __name__ == "__main__":
    main()
