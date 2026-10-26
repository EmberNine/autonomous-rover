#!/usr/bin/env python3
"""Strict parser for config/vehicle.cfg, reusable by ROS 2 launch files.

This mirrors the line-parsing rules of the C++ core's
``rover::VehicleConfig::loadFromFile`` (repo root ``src/vehicle_config.cpp``)
exactly, so the two parsers never disagree about what a line in
``vehicle.cfg`` means: '#' starts a comment (whole-line or trailing), only
" \\t\\r\\n" count as whitespace, every non-blank line must contain '=', and
an unrecognized key is silently ignored.

Where this module is deliberately *stricter* than the C++ core: the URDF
cannot render sensibly with a missing or zero dimension, so the eight values
rover.urdf.xacro needs are treated here as required and strictly positive.
The C++ core instead leaves missing numeric keys at a 0.0 default, because
its bicycle model does not need every field to be present.
"""

import math
from pathlib import Path
from typing import Dict, NamedTuple, Union

# The eight xacro arguments rover.urdf.xacro requires (see its <xacro:arg>
# block). Every other key vehicle.cfg may contain -- including "name", a
# string the C++ core stores but this module has no use for, and
# "meters_per_encoder_tick", not read by the URDF -- is parsed but then
# ignored here, exactly like an unrecognized key.
REQUIRED_URDF_KEYS = (
    "wheelbase_meters",
    "front_track_width_meters",
    "rear_track_width_meters",
    "tire_diameter_meters",
    "max_steering_angle_radians",
    "vehicle_mass_kilograms",
    "front_wheel_mass_kilograms",
    "rear_wheel_mass_kilograms",
)

# The characters the C++ core's trimWhitespace() treats as whitespace. Kept
# as an explicit constant rather than Python's default str.strip() (which
# also strips other whitespace, e.g. \v, \f) so both parsers agree exactly.
_WHITESPACE_CHARACTERS = " \t\r\n"


class VehicleConfigError(RuntimeError):
    """Raised when vehicle.cfg cannot be read or fails validation.

    Every message names both the offending key (where applicable) and the
    config file path, so a launch failure points straight at the fix.
    """


def _strip_comment_and_surrounding_whitespace(raw_line: str) -> str:
    """Drop everything from the first '#' onward, then trim whitespace.

    Mirrors trimWhitespace() and the comment handling in
    src/vehicle_config.cpp: a '#' may start a whole-line comment or trail
    after a value on the same line, and what remains is trimmed of
    " \\t\\r\\n" only.
    """
    comment_start_index = raw_line.find("#")
    if comment_start_index != -1:
        raw_line = raw_line[:comment_start_index]
    return raw_line.strip(_WHITESPACE_CHARACTERS)


def _parse_key_value_lines(config_file_path: Path) -> Dict[str, str]:
    """Read `key = value` lines into a dict of raw (unparsed) string values.

    Blank lines and comment-only lines are skipped. A non-comment line
    without '=' is malformed and raises VehicleConfigError, matching the
    C++ parser's error. Values come back as trimmed strings, not yet
    converted to numbers, so the "name" key and any unrecognized key can
    pass through without a numeric-parse attempt. If a key repeats, the
    later occurrence wins (a plain dict assignment already does this,
    matching the C++ parser's line-by-line overwrite).
    """
    if not config_file_path.is_file():
        raise VehicleConfigError(f"Cannot open vehicle config file: {config_file_path}")

    raw_values_by_key: Dict[str, str] = {}
    with config_file_path.open("r", encoding="utf-8") as config_file:
        for line_number, raw_line in enumerate(config_file, start=1):
            line_without_comment = _strip_comment_and_surrounding_whitespace(raw_line)
            if not line_without_comment:
                continue

            equals_sign_index = line_without_comment.find("=")
            if equals_sign_index == -1:
                raise VehicleConfigError(
                    f"{config_file_path}:{line_number}: expected 'key = value', "
                    f"got '{line_without_comment}'"
                )

            key = line_without_comment[:equals_sign_index].strip(_WHITESPACE_CHARACTERS)
            value_text = line_without_comment[equals_sign_index + 1 :].strip(
                _WHITESPACE_CHARACTERS
            )
            raw_values_by_key[key] = value_text

    return raw_values_by_key


def _parse_required_positive_float(
    raw_values_by_key: Dict[str, str], key: str, config_file_path: Path
) -> float:
    """Look up `key` and parse it as a finite, strictly positive float.

    Raises VehicleConfigError naming both the key and the file path if the
    key is absent, its value doesn't parse as a finite number, or the
    parsed value is zero or negative.
    """
    if key not in raw_values_by_key:
        raise VehicleConfigError(f"{config_file_path}: missing required key '{key}'")

    value_text = raw_values_by_key[key]

    # Python's float() accepts digit-group underscores ("1_000.5"), a
    # Python-only literal syntax that C++'s std::stod does not understand.
    # Reject it explicitly so the two parsers can never read the same
    # value differently.
    if "_" in value_text:
        raise VehicleConfigError(
            f"{config_file_path}: key '{key}' has non-numeric value '{value_text}'"
        )

    try:
        parsed_value = float(value_text)
    except ValueError as parse_error:
        raise VehicleConfigError(
            f"{config_file_path}: key '{key}' has non-numeric value '{value_text}'"
        ) from parse_error

    # float() parses "inf"/"nan" successfully; std::stod does too, which is
    # exactly why the C++ core follows up with an isfinite() check. Mirror
    # that here.
    if not math.isfinite(parsed_value):
        raise VehicleConfigError(
            f"{config_file_path}: key '{key}' has non-numeric value '{value_text}'"
        )

    if not parsed_value > 0.0:
        raise VehicleConfigError(
            f"{config_file_path}: key '{key}' must be strictly positive, "
            f"got {parsed_value}"
        )

    return parsed_value


class VehicleUrdfDimensions(NamedTuple):
    """The eight vehicle.cfg values rover.urdf.xacro requires, as floats."""

    wheelbase_meters: float
    front_track_width_meters: float
    rear_track_width_meters: float
    tire_diameter_meters: float
    max_steering_angle_radians: float
    vehicle_mass_kilograms: float
    front_wheel_mass_kilograms: float
    rear_wheel_mass_kilograms: float


def load_vehicle_urdf_dimensions(
    config_file_path: Union[str, Path]
) -> VehicleUrdfDimensions:
    """Parse vehicle.cfg and return the eight URDF dimensions as floats.

    Args:
        config_file_path: Path to a vehicle.cfg-formatted file.

    Returns:
        A VehicleUrdfDimensions namedtuple with all eight values as
        positive, finite floats.

    Raises:
        VehicleConfigError: if the file cannot be opened, contains a
            malformed (non-comment, no '=') line, or any of
            REQUIRED_URDF_KEYS is missing, non-numeric, non-finite, or not
            strictly positive. The message names the offending key and
            the file path.
    """
    config_file_path = Path(config_file_path)
    raw_values_by_key = _parse_key_value_lines(config_file_path)
    return VehicleUrdfDimensions(
        *(
            _parse_required_positive_float(raw_values_by_key, key, config_file_path)
            for key in REQUIRED_URDF_KEYS
        )
    )
