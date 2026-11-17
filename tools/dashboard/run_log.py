"""Local archive of Buddy 4 run summaries."""
from contextlib import closing
import sqlite3


MAX_RUN_LOGS = 40


class RunLog:
    def __init__(self, folder):
        self.path = folder / "buddy4-runs.sqlite3"
        self.records = {}
        self._pending = {}
        self.error = None
        if self.path.is_file():
            try:
                with closing(self._connect()) as db:
                    rows = db.execute(
                        "SELECT * FROM runs ORDER BY started_at DESC, rowid DESC LIMIT ?",
                        (MAX_RUN_LOGS,),
                    ).fetchall()
                self.records = {row["id"]: dict(row) for row in reversed(rows)}
            except sqlite3.Error as exc:
                self.error = f"Could not load saved runs. {exc}"

    def _connect(self):
        db = sqlite3.connect(self.path, timeout=0.05)
        db.row_factory = sqlite3.Row
        return db

    def record(self, run):
        record = run.copy()
        self.records[run["id"]] = self._pending[run["id"]] = record
        if len(self.records) > MAX_RUN_LOGS:
            oldest_id = next(iter(self.records))
            self.records.pop(oldest_id)
            self._pending.pop(oldest_id, None)
        self._flush()

    def _flush(self):
        if not self._pending:
            return
        try:
            with closing(self._connect()) as db, db:
                db.execute("""CREATE TABLE IF NOT EXISTS runs (
                    id TEXT PRIMARY KEY, started_at TEXT NOT NULL, ended_at TEXT,
                    hump_count INTEGER, peak_mm INTEGER)""")
                db.executemany("""INSERT INTO runs (id, started_at, ended_at, hump_count, peak_mm)
                    VALUES (:id, :started_at, :ended_at, :hump_count, :peak_mm)
                    ON CONFLICT(id) DO UPDATE SET ended_at=excluded.ended_at,
                    hump_count=excluded.hump_count, peak_mm=excluded.peak_mm""", self._pending.values())
            self._pending.clear()
            self.error = None
        except sqlite3.Error as exc:
            self.error = f"Could not save a run to disk. {exc}"

    def snapshot(self, current_id, active):
        self._flush()
        return dict(runs=[dict(run, active=active and run["id"] == current_id)
                          for run in reversed(self.records.values())],
                    current_id=current_id, error=self.error)
