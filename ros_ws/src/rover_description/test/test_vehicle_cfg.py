#!/usr/bin/env python3
"""Standalone pytest tests for launch/vehicle_cfg.py.

Run from this package's directory with `python3 -m pytest`. Deliberately
not wired into ament_cmake_pytest / colcon test: it's a plain, fast unit
suite for a single parser module, and colcon-level testing is not part of
this package's build.
"""

import sys
from pathlib import Path

import pytest

# vehicle_cfg.py lives beside the launch files, not in an importable
# package (this is an ament_cmake package with no setup.py), so its
# directory must be added to sys.path before it can be imported.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "launch"))

from vehicle_cfg import (  # noqa: E402  (import must follow the sys.path edit above)
    VehicleConfigError,
    load_vehicle_urdf_dimensions,
)


VALID_CONFIG_TEXT = (
    "name = Test Rig\n"
    "wheelbase_meters = 0.133\n"
    "front_track_width_meters = 0.083\n"
    "rear_track_width_meters = 0.081\n"
    "tire_diameter_meters = 0.054\n"
    "meters_per_encoder_tick = 1.0\n"
    "max_steering_angle_radians = 0.246\n"
    "vehicle_mass_kilograms = 0.302\n"
    "front_wheel_mass_kilograms = 0.020\n"
    "rear_wheel_mass_kilograms = 0.011\n"
)

# Every required numeric key's exact "key = value\n" line in
# VALID_CONFIG_TEXT, used to build "missing this one key" fixtures below.
REQUIRED_KEY_LINES = (
    "wheelbase_meters = 0.133\n",
    "front_track_width_meters = 0.083\n",
    "rear_track_width_meters = 0.081\n",
    "tire_diameter_meters = 0.054\n",
    "max_steering_angle_radians = 0.246\n",
    "vehicle_mass_kilograms = 0.302\n",
    "front_wheel_mass_kilograms = 0.020\n",
    "rear_wheel_mass_kilograms = 0.011\n",
)


def _write_config(tmp_path: Path, contents: str) -> Path:
    """Write `contents` to a vehicle.cfg file under tmp_path and return it."""
    config_file_path = tmp_path / "vehicle.cfg"
    config_file_path.write_text(contents)
    return config_file_path


def test_valid_file_returns_all_eight_dimensions(tmp_path):
    config_file_path = _write_config(tmp_path, VALID_CONFIG_TEXT)

    dimensions = load_vehicle_urdf_dimensions(config_file_path)

    assert dimensions.wheelbase_meters == pytest.approx(0.133)
    assert dimensions.front_track_width_meters == pytest.approx(0.083)
    assert dimensions.rear_track_width_meters == pytest.approx(0.081)
    assert dimensions.tire_diameter_meters == pytest.approx(0.054)
    assert dimensions.max_steering_angle_radians == pytest.approx(0.246)
    assert dimensions.vehicle_mass_kilograms == pytest.approx(0.302)
    assert dimensions.front_wheel_mass_kilograms == pytest.approx(0.020)
    assert dimensions.rear_wheel_mass_kilograms == pytest.approx(0.011)


def test_missing_key_raises_and_names_the_key(tmp_path):
    config_without_mass = VALID_CONFIG_TEXT.replace("vehicle_mass_kilograms = 0.302\n", "")
    config_file_path = _write_config(tmp_path, config_without_mass)

    with pytest.raises(VehicleConfigError, match="vehicle_mass_kilograms"):
        load_vehicle_urdf_dimensions(config_file_path)


@pytest.mark.parametrize("required_key_line", REQUIRED_KEY_LINES)
def test_each_required_key_is_required(tmp_path, required_key_line):
    config_file_path = _write_config(
        tmp_path, VALID_CONFIG_TEXT.replace(required_key_line, "")
    )

    with pytest.raises(VehicleConfigError, match=required_key_line.split(" =")[0]):
        load_vehicle_urdf_dimensions(config_file_path)


def test_non_numeric_value_raises(tmp_path):
    config_file_path = _write_config(
        tmp_path, VALID_CONFIG_TEXT.replace("0.133", "five point two")
    )

    with pytest.raises(VehicleConfigError, match="wheelbase_meters"):
        load_vehicle_urdf_dimensions(config_file_path)


def test_trailing_garbage_after_number_raises(tmp_path):
    config_file_path = _write_config(
        tmp_path, VALID_CONFIG_TEXT.replace("0.133", "0.133abc")
    )

    with pytest.raises(VehicleConfigError, match="wheelbase_meters"):
        load_vehicle_urdf_dimensions(config_file_path)


def test_malformed_line_without_equals_sign_raises(tmp_path):
    config_file_path = _write_config(tmp_path, VALID_CONFIG_TEXT + "this line has no equals\n")

    with pytest.raises(VehicleConfigError, match="expected 'key = value'"):
        load_vehicle_urdf_dimensions(config_file_path)


def test_comments_and_blank_lines_are_ignored(tmp_path):
    config_text = (
        "# a full-line comment\n"
        "\n"
        "   \t  \n"
        "wheelbase_meters = 0.133   # trailing comment\n"
        "# wheelbase_meters = 99.0   (commented out, must not apply)\n"
        "front_track_width_meters = 0.083\n"
        "rear_track_width_meters = 0.081\n"
        "tire_diameter_meters = 0.054\n"
        "max_steering_angle_radians = 0.246\n"
        "vehicle_mass_kilograms = 0.302\n"
        "front_wheel_mass_kilograms = 0.020\n"
        "rear_wheel_mass_kilograms = 0.011\n"
    )
    config_file_path = _write_config(tmp_path, config_text)

    dimensions = load_vehicle_urdf_dimensions(config_file_path)

    assert dimensions.wheelbase_meters == pytest.approx(0.133)


def test_unknown_keys_are_ignored(tmp_path):
    config_file_path = _write_config(
        tmp_path, VALID_CONFIG_TEXT + "flux_capacitor_gigawatts = 1.21\npaint_color = olive\n"
    )

    dimensions = load_vehicle_urdf_dimensions(config_file_path)

    assert dimensions.wheelbase_meters == pytest.approx(0.133)


@pytest.mark.parametrize("bad_value", ["0", "-0.1", "-1"])
def test_zero_or_negative_value_is_rejected(tmp_path, bad_value):
    config_file_path = _write_config(
        tmp_path, VALID_CONFIG_TEXT.replace("wheelbase_meters = 0.133", f"wheelbase_meters = {bad_value}")
    )

    with pytest.raises(VehicleConfigError, match="strictly positive"):
        load_vehicle_urdf_dimensions(config_file_path)


def test_missing_file_raises(tmp_path):
    nonexistent_path = tmp_path / "does_not_exist.cfg"

    with pytest.raises(VehicleConfigError, match="Cannot open"):
        load_vehicle_urdf_dimensions(nonexistent_path)


def test_later_duplicate_key_wins(tmp_path):
    config_file_path = _write_config(
        tmp_path,
        VALID_CONFIG_TEXT.replace(
            "wheelbase_meters = 0.133\n",
            "wheelbase_meters = 0.1\nwheelbase_meters = 0.2\n",
        ),
    )

    dimensions = load_vehicle_urdf_dimensions(config_file_path)

    assert dimensions.wheelbase_meters == pytest.approx(0.2)


def test_underscore_digit_grouping_is_rejected(tmp_path):
    # Python's float() accepts "0_133" as digit-group underscores (a
    # Python-only literal syntax); std::stod does not. Reject it so the two
    # parsers can never read this value differently.
    config_file_path = _write_config(
        tmp_path, VALID_CONFIG_TEXT.replace("0.133", "0_133")
    )

    with pytest.raises(VehicleConfigError, match="wheelbase_meters"):
        load_vehicle_urdf_dimensions(config_file_path)


@pytest.mark.parametrize("non_finite_value", ["nan", "inf", "-inf", "infinity"])
def test_non_finite_value_is_rejected(tmp_path, non_finite_value):
    # float() parses all of these successfully (matching std::stod, which
    # also accepts "nan"/"inf" spellings), so isfinite() must reject them
    # explicitly rather than relying on the parse itself failing.
    config_file_path = _write_config(
        tmp_path, VALID_CONFIG_TEXT.replace("0.133", non_finite_value)
    )

    with pytest.raises(VehicleConfigError, match="wheelbase_meters"):
        load_vehicle_urdf_dimensions(config_file_path)
