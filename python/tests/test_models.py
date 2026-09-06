"""Typed measurement access uses the same data as JSONL serialization."""

import json
import math

import pytest

from hipnuc.models import Sample


VECTOR_FIELDS = {
    "acceleration_m_s2": 3,
    "angular_velocity_rad_s": 3,
    "magnetic_field_t": 3,
    "euler_rad": 3,
    "quaternion_wxyz": 4,
    "velocity_enu_m_s": 3,
    "heave_surge_sway_m": 3,
    "heave_surge_sway_hz": 3,
    "inclination_rad": 2,
}
SCALAR_FIELDS = (
    "roll_rad",
    "pitch_rad",
    "heading_rad",
    "latitude_deg",
    "longitude_deg",
    "altitude_msl_m",
    "geoid_separation_m",
    "pressure_pa",
    "temperature_c",
)


@pytest.mark.parametrize("name", [*VECTOR_FIELDS, *SCALAR_FIELDS])
def test_missing_measurement_remains_unavailable(name):
    sample = Sample("HI91", {}, b"")
    assert getattr(sample, name) is None
    assert name not in sample.to_dict()
    sample.values[name] = None
    assert getattr(sample, name) is None
    assert sample.to_dict()[name] is None


@pytest.mark.parametrize("name,size", VECTOR_FIELDS.items())
def test_vector_property_matches_jsonl_without_duplicating_storage(name, size):
    original = list(range(size))
    sample = Sample("HI83", {name: original}, b"")
    assert getattr(sample, name) == tuple(original)
    assert isinstance(getattr(sample, name), tuple)
    assert list(getattr(sample, name)) == sample.to_dict()[name]
    assert sample.values[name] is original
    original[0] = 42
    assert getattr(sample, name)[0] == 42
    assert sample.to_dict()[name][0] == 42


@pytest.mark.parametrize("name", SCALAR_FIELDS)
def test_scalar_property_matches_jsonl_and_reads_current_values(name):
    sample = Sample("HI83", {name: 1.25}, b"")
    assert getattr(sample, name) == sample.to_dict()[name] == 1.25
    sample.values[name] = 2.5
    assert getattr(sample, name) == sample.to_dict()[name] == 2.5


@pytest.mark.parametrize("invalid", [None, math.nan, math.inf, -math.inf])
def test_nonfinite_components_preserve_other_vector_axes(invalid):
    sample = Sample("HI91", {"acceleration_m_s2": [1.0, invalid, 3.0]}, b"")
    assert sample.acceleration_m_s2 == (1.0, None, 3.0)
    assert sample.to_dict()["acceleration_m_s2"] == [1.0, None, 3.0]
    json.dumps(sample.to_dict(), allow_nan=False)
    assert sample.values["acceleration_m_s2"][1] is invalid


@pytest.mark.parametrize("name", SCALAR_FIELDS)
@pytest.mark.parametrize("invalid", [math.nan, math.inf, -math.inf])
def test_nonfinite_scalars_are_unavailable_without_changing_values(name, invalid):
    sample = Sample("HI83", {name: invalid}, b"")
    assert getattr(sample, name) is None
    assert sample.to_dict()[name] is None
    assert sample.values[name] is invalid


def test_heading_yaw_and_protocol_specific_fields_remain_distinct():
    sample = Sample(
        "HI83", {"heading_rad": 1.2, "euler_rad": [0.1, 0.2, 0.3], "main_status": 123}, b""
    )
    assert sample.heading_rad == 1.2
    assert sample.euler_rad == (0.1, 0.2, 0.3)
    assert sample.values["main_status"] == 123
    with pytest.raises(AttributeError):
        _ = sample.main_status
