from __future__ import annotations

from builtin_interfaces.msg import Time
from std_msgs.msg import Header


def coerce_time(value) -> Time:
    """Return a ROS Time message from a Time or lightweight test fixture."""
    if isinstance(value, Time):
        return value
    out = Time()
    out.sec = int(getattr(value, "sec", 0))
    out.nanosec = int(getattr(value, "nanosec", 0))
    return out


def coerce_header(value) -> Header:
    """Return a ROS Header message from a Header or header-like object."""
    if isinstance(value, Header):
        return value
    out = Header()
    out.stamp = coerce_time(getattr(value, "stamp", None))
    out.frame_id = str(getattr(value, "frame_id", ""))
    return out
