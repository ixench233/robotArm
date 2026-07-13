import json
from pathlib import Path
from typing import Dict, List


class ActionStore:
    def __init__(self, root: str):
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)

    def list_sequences(self) -> List[str]:
        return sorted(path.stem for path in self.root.glob("*.json"))

    def load(self, name: str) -> List[Dict]:
        path = self._path(name)
        if not path.exists():
            raise FileNotFoundError(name)
        return json.loads(path.read_text(encoding="utf-8"))

    def save(self, name: str, steps: List[Dict]):
        path = self._path(name)
        path.write_text(json.dumps(steps, ensure_ascii=False, indent=2), encoding="utf-8")

    def _path(self, name: str) -> Path:
        safe = "".join(ch for ch in name if ch.isalnum() or ch in ("-", "_")).strip()
        if not safe:
            safe = "sequence"
        return self.root / f"{safe}.json"

