"""Source-level scene composition for the semantic-navigation Isaac Sim port.

This package deliberately performs no Isaac Sim imports at module import time.  The
standalone entry point creates :class:`isaacsim.SimulationApp` before importing the
runtime builder; pure-Python manifest and geometry helpers remain testable without
Isaac Sim installed.
"""

from .manifest import ManifestError, load_manifest, validate_manifest

__all__ = ["ManifestError", "load_manifest", "validate_manifest"]
