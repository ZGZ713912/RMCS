"""Consumer for a validated count-domain fit exported by fit_current_sweep.py."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np


class CurrentActuatorModel:
    def __init__(self, path: Path, joint: str, condition: str):
        document = json.loads(Path(path).read_text())
        if document.get("command_domain") != "raw_current" or document.get("schema_version") != 1:
            raise ValueError("Model must use the current-count schema")
        self.model = document["models"][joint]
        if self.model["condition"] != condition or not self.model["training_ready"]:
            raise ValueError("Require a validated fit for this suspension/contact condition")
        if self.model["absolute_torque_identified"]:
            raise ValueError("This consumer does not accept absolute-torque labels")
        self.coefficients = np.asarray(self.model["coefficients"], dtype=float)
        self.command_limit = float(document["source_metadata"]["max_command_current_raw"])
        self.reset()

    def reset(self, current_raw: float = 0.0):
        if not np.isfinite(current_raw):
            raise ValueError("Initial current must be finite")
        self.current_raw = float(current_raw)
        self.time_s = 0.0
        self.history = [(0.0, 0.0)]

    def step_current(self, command_raw: float, dt: float) -> float:
        if not np.isfinite(command_raw) or not np.isfinite(dt) or dt <= 0:
            raise ValueError("Finite command and positive time step required")
        clipped = np.clip(command_raw, -self.command_limit, self.command_limit)
        command = float(np.sign(clipped) * np.floor(abs(clipped) + 0.5))
        self.history.append((self.time_s, command))
        self.time_s += dt
        tr = self.model["transport"]
        delayed = self.time_s - 0.5 * dt - tr["delay_s"]
        # Retain the entry preceding the delayed time; this is a causal ZOH delay.
        while len(self.history) > 1 and self.history[1][0] <= delayed:
            self.history.pop(0)
        u = self.history[0][1]
        a = np.exp(-dt / tr["time_constant_s"]) if tr["time_constant_s"] > 0 else 0.0
        self.current_raw = float(a * self.current_raw + (1 - a) * (tr["gain"] * u + tr["bias_raw"]))
        return self.current_raw

    def acceleration(self, angle: np.ndarray, velocity: np.ndarray, current_raw: np.ndarray) -> np.ndarray:
        q, v, iq = np.broadcast_arrays(angle, velocity, current_raw)
        if not np.all(np.isfinite(q)) or not np.all(np.isfinite(v)) or not np.all(np.isfinite(iq)):
            raise ValueError("State and measured-current inputs must be finite")
        coverage = self.model["coverage"]
        for values, key in ((q, "angle_rad"), (v, "velocity_rad_s"), (iq, "feedback_current_raw")):
            low, high = coverage[key]
            if np.any(values < low) or np.any(values > high):
                raise ValueError(f"State outside measured {key} coverage; collect another sweep")
        f = np.stack((iq, iq * (q - self.model["angle_center_rad"]), iq * np.abs(v), v,
                      np.tanh(10 * v), np.sin(q), np.cos(q), np.ones_like(q)), axis=-1)
        return f @ self.coefficients

    def equivalent_simulation_torque(self, angle, velocity, current_raw,
                                     simulation_inertia, passive_simulation_torque):
        """Torque which matches fitted acceleration under a supplied simulator model.

        J must be the configuration-dependent effective inertia (or externally computed
        multibody mapping). Passive/contact torque comes from that same simulation.
        The result is a simulation-equivalent torque, not measured real shaft torque.
        """
        if not np.all(np.isfinite(simulation_inertia)) or np.any(np.asarray(simulation_inertia) <= 0):
            raise ValueError("Supply positive effective inertia from the simulator")
        if not np.all(np.isfinite(passive_simulation_torque)):
            raise ValueError("Supply finite passive/contact torque from the simulator")
        return (np.asarray(simulation_inertia) * self.acceleration(angle, velocity, current_raw)
                - np.asarray(passive_simulation_torque))
