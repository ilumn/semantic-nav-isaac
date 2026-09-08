from __future__ import annotations

from pathlib import Path

import yaml

from .config import ROOT_DIR


class Ontology:
    def __init__(self, payload: dict[str, dict]):
        self.payload = payload

    @classmethod
    def from_default_file(cls) -> "Ontology":
        path = ROOT_DIR / "semantic_nav_memory" / "ontology.yaml"
        with path.open("r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
        return cls(data)

    def node(self, label: str) -> dict:
        return self.payload.get(label, {})

    def aliases(self, label: str) -> list[str]:
        return list(self.node(label).get("aliases", []))

    def parents(self, label: str) -> list[str]:
        return list(self.node(label).get("parents", []))

    def mobility(self, label: str) -> str:
        return str(self.node(label).get("mobility", "unknown"))

    def support_priors(self, label: str) -> list[str]:
        return list(self.node(label).get("support_priors", []))

    def place_priors(self, label: str) -> list[str]:
        return list(self.node(label).get("place_priors", []))

    @property
    def path(self) -> Path:
        return ROOT_DIR / "semantic_nav_memory" / "ontology.yaml"
