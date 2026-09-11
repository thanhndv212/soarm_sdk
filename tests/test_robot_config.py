"""Unit tests for soarm_sdk.robot.config.validate_robot_config."""

from __future__ import annotations

import pytest

from soarm_sdk.robot.config import ConfigError, validate_robot_config


def _valid_config(**overrides):
    cfg = {
        "n_dof": 3,
        "joint_names": ["a", "b", "c"],
        "home_position": [0.0, 0.0, 0.0],
        "joint_limits_lower": [-1.0, -1.0, -1.0],
        "joint_limits_upper": [1.0, 1.0, 1.0],
    }
    cfg.update(overrides)
    return cfg


def test_valid_config_raises_nothing():
    validate_robot_config(_valid_config())  # should not raise


def test_missing_required_key_raises():
    cfg = _valid_config()
    del cfg["home_position"]
    with pytest.raises(ConfigError, match="home_position"):
        validate_robot_config(cfg)


def test_missing_multiple_keys_lists_all_of_them():
    cfg = _valid_config()
    del cfg["home_position"]
    del cfg["joint_names"]
    with pytest.raises(ConfigError) as excinfo:
        validate_robot_config(cfg)
    assert "home_position" in str(excinfo.value)
    assert "joint_names" in str(excinfo.value)


def test_non_positive_n_dof_raises():
    with pytest.raises(ConfigError, match="n_dof"):
        validate_robot_config(_valid_config(n_dof=0))


def test_wrong_length_array_raises():
    with pytest.raises(ConfigError, match="joint_names"):
        validate_robot_config(_valid_config(joint_names=["a", "b"]))


def test_lower_limit_exceeding_upper_limit_raises():
    with pytest.raises(ConfigError, match="joint_limits_lower"):
        validate_robot_config(
            _valid_config(
                joint_limits_lower=[0.0, 0.0, 2.0],
                joint_limits_upper=[1.0, 1.0, 1.0],
            )
        )


def test_non_sequence_field_raises():
    with pytest.raises(ConfigError, match="joint_names"):
        validate_robot_config(_valid_config(joint_names=None))
