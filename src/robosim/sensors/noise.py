"""Shared two-stage noise model used by all sensors."""

from __future__ import annotations

import math
import random

from robosim.config import NoiseConfig


class NoisePipeline:
    """Shared two-stage noise model used by all sensors.

    Each call to :meth:`apply` adds:

    1. Instantaneous Gaussian noise: ``gauss(0, σ_noise)``
    2. Accumulated bias drift: a random walk that grows over time

    The bias drift step is scaled by ``√dt`` so that noise statistics are
    independent of the simulation timestep.

    With default ``NoiseConfig`` (all zeros) this is a pass-through.
    """

    def __init__(self, config: NoiseConfig) -> None:
        self._gaussian_sigma = config.gaussian_sigma
        self._drift_sigma = config.bias_drift_sigma
        self._accumulated_bias: float = 0.0

    @property
    def accumulated_bias(self) -> float:
        return self._accumulated_bias

    def apply(self, true_value: float, dt: float) -> float:
        """Return a noisy reading from *true_value*."""
        # Update bias random walk
        if self._drift_sigma:
            self._accumulated_bias += random.gauss(0.0, self._drift_sigma * math.sqrt(dt))

        noise = 0.0
        if self._gaussian_sigma:
            noise = random.gauss(0.0, self._gaussian_sigma)

        return true_value + noise + self._accumulated_bias

    def reset(self) -> None:
        self._accumulated_bias = 0.0
