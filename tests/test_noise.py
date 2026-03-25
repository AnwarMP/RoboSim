"""Tests for the shared noise pipeline."""

from __future__ import annotations

from robosim.config import NoiseConfig
from robosim.sensors import NoisePipeline


class TestNoisePipeline:
    """Unit tests for the shared noise model."""

    def test_zero_noise_is_passthrough(self) -> None:
        pipe = NoisePipeline(NoiseConfig())
        assert pipe.apply(42.0, 1.0 / 60) == 42.0
        assert pipe.apply(-3.5, 1.0 / 60) == -3.5

    def test_zero_noise_no_bias_drift(self) -> None:
        pipe = NoisePipeline(NoiseConfig())
        for _ in range(1000):
            pipe.apply(0.0, 1.0 / 60)
        assert pipe.accumulated_bias == 0.0

    def test_gaussian_noise_has_nonzero_variance(self) -> None:
        import random as _rng

        _rng.seed(12345)
        pipe = NoisePipeline(NoiseConfig(gaussian_sigma=1.0))
        readings = [pipe.apply(0.0, 1.0 / 60) for _ in range(500)]
        # With σ=1, readings should scatter around 0
        assert max(readings) > 0.5
        assert min(readings) < -0.5

    def test_bias_drift_accumulates_over_time(self) -> None:
        import random as _rng

        _rng.seed(99)
        pipe = NoisePipeline(NoiseConfig(bias_drift_sigma=0.1))
        for _ in range(600):
            pipe.apply(0.0, 1.0 / 60)
        # After 10 simulated seconds of drift, bias should be nonzero
        assert pipe.accumulated_bias != 0.0

    def test_reset_clears_bias(self) -> None:
        import random as _rng

        _rng.seed(42)
        pipe = NoisePipeline(NoiseConfig(bias_drift_sigma=0.5))
        for _ in range(100):
            pipe.apply(0.0, 1.0 / 60)
        assert pipe.accumulated_bias != 0.0
        pipe.reset()
        assert pipe.accumulated_bias == 0.0
