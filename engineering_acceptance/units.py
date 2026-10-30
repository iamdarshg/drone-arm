"""Small explicit unit registry. Values are converted through SI base units."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Unit:
    dimension: str
    to_si: float
    offset: float = 0.0


UNITS = {
    "1": Unit("dimensionless", 1), "m": Unit("length", 1), "mm": Unit("length", .001),
    "kg": Unit("mass", 1), "g": Unit("mass", .001), "s": Unit("time", 1),
    "A": Unit("current", 1), "V": Unit("voltage", 1), "W": Unit("power", 1),
    "Ah": Unit("charge", 3600), "ohm": Unit("resistance", 1),
    "Wh": Unit("energy", 3600), "J": Unit("energy", 1), "K": Unit("temperature", 1),
    "C": Unit("temperature", 1, 273.15), "degC": Unit("temperature", 1, 273.15), "rad": Unit("angle", 1), "deg": Unit("angle", 0.017453292519943295),
    "Hz": Unit("frequency", 1), "rpm": Unit("frequency", 1/60), "m2": Unit("area", 1),
    "mm2": Unit("area", 1e-6), "kg*m2": Unit("inertia", 1), "kg/m3": Unit("density", 1), "Pa": Unit("pressure", 1),
    "USD": Unit("currency_USD", 1),
    # Typed catalog labels, not arithmetic conversion units.
}


def convert(value: float, source: str, target: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError("conversion value must be finite")
    if source not in UNITS or target not in UNITS:
        raise ValueError(f"unsupported unit conversion {source!r} -> {target!r}")
    a, b = UNITS[source], UNITS[target]
    if a.dimension != b.dimension:
        raise ValueError(f"incompatible units {source!r} ({a.dimension}) and {target!r} ({b.dimension})")
    converted=(value * a.to_si + a.offset - b.offset) / b.to_si
    if not math.isfinite(converted): raise ValueError("converted result is not finite")
    return converted
