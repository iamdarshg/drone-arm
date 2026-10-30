"""Unit algebra properties; generated numeric domains are not operating envelopes."""
import math
import pytest
from hypothesis import given, settings, strategies as st
from engineering_acceptance.units import convert

finite = st.floats(min_value=-1e6, max_value=1e6, allow_nan=False, allow_infinity=False)

@settings(max_examples=100, derandomize=True, deadline=None)
@given(finite)
def test_length_round_trip(value):
    assert math.isclose(convert(convert(value, "mm", "m"), "m", "mm"), value, rel_tol=1e-12, abs_tol=1e-9)

@settings(max_examples=100, derandomize=True, deadline=None)
@given(st.floats(min_value=0, max_value=1e6, allow_nan=False, allow_infinity=False))
def test_energy_is_additive_with_explicit_units(value):
    assert math.isclose(convert(value, "Wh", "J"), value * 3600, rel_tol=1e-12, abs_tol=1e-9)
    with pytest.raises(ValueError):
        convert(value, "W", "Wh")

@settings(max_examples=100, derandomize=True, deadline=None)
@given(st.floats(min_value=-273.15, max_value=1000, allow_nan=False, allow_infinity=False))
def test_temperature_affine_round_trip(value):
    kelvin = convert(value, "degC", "K")
    assert kelvin >= 0
    assert math.isclose(convert(kelvin, "K", "degC"), value, rel_tol=1e-12, abs_tol=1e-10)
