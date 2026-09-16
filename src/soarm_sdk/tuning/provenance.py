"""Persist what an auto-tune run did, next to the arm's calibration file.

A tuned joint's gains should be as auditable as its zero calibration: which
algorithm, which criteria, what it started from, what it ended at, and
whether the result was ever accepted. EEPROM itself has no room for this —
it only holds the three gain bytes — so it lives in its own JSON file,
mirroring ``~/.soarm_sdk/calibration.json``'s one-file-per-arm convention.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

from .gains_io import Gains
from .search import SearchResult

__all__ = ["TuningRecord", "DEFAULT_PROVENANCE_PATH", "load_records", "append_record"]

#: Mirrors the calibration module's ``Path.home() / ".soarm_sdk" / ...``
#: convention. Tests must not rely on this default — see
#: ``tests/conftest.py``'s autouse fixture, which redirects it.
DEFAULT_PROVENANCE_PATH = Path.home() / ".soarm_sdk" / "pid_tuning.json"


@dataclass(frozen=True)
class TuningRecord:
    arm_id: str
    servo_id: int
    joint_name: str
    algorithm: str
    before: Gains
    after: Gains
    validated: bool
    exhausted: bool
    trial_count: int
    timestamp: str = field(
        default_factory=lambda: datetime.now(timezone.utc).isoformat()
    )
    notes: Dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_search_result(
        cls,
        *,
        arm_id: str,
        servo_id: int,
        joint_name: str,
        algorithm: str,
        before: Gains,
        result: SearchResult,
        notes: Optional[Dict[str, Any]] = None,
    ) -> "TuningRecord":
        return cls(
            arm_id=arm_id,
            servo_id=servo_id,
            joint_name=joint_name,
            algorithm=algorithm,
            before=before,
            after=result.best.gains,
            validated=result.validated,
            exhausted=result.exhausted,
            trial_count=len(result.trials),
            notes=notes or {},
        )

    def as_dict(self) -> Dict[str, Any]:
        return asdict(self)


def load_records(path: Path = DEFAULT_PROVENANCE_PATH) -> List[TuningRecord]:
    """Return every record on file, oldest first. Empty if the file doesn't exist."""
    if not path.exists():
        return []
    raw = json.loads(path.read_text())
    records = []
    for entry in raw:
        entry = dict(entry)
        entry["before"] = Gains(**entry["before"])
        entry["after"] = Gains(**entry["after"])
        records.append(TuningRecord(**entry))
    return records


def append_record(record: TuningRecord, path: Path = DEFAULT_PROVENANCE_PATH) -> Path:
    """Append *record* to the provenance file, creating it if needed."""
    records = load_records(path)
    records.append(record)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps([r.as_dict() for r in records], indent=2) + "\n")
    return path
