"""Local archive of observed robot runs and decoded telemetry."""
from collections import deque
from contextlib import closing
import json
import sqlite3
import time


MAX_RUN_LOGS = 20
MAX_ENTRIES_PER_STREAM = 1200
CHECKPOINT_S = 1.0

_RUN_COLUMNS = """
    r.id, r.started_at, r.ended_at, r.hump_count, r.peak_mm, r.end_reason, r.log_version,
    EXISTS(SELECT 1 FROM run_entries WHERE run_id=r.id) AS has_details
"""


class RunLog:
    def __init__(self, folder, *, clock=time.monotonic):
        self.path = folder / "buddy4-runs.sqlite3"
        self.clock = clock
        self.records = {}
        self.error = None
        self._initialized = False
        self._open_runs = set()
        self._pending_runs = {}
        self._pending_entries = {}
        self._pending_dropped = {}
        self._sequence = 0
        self._last_checkpoint = self.clock()
        if self.path.is_file():
            self._flush()

    def _connect(self):
        db = sqlite3.connect(self.path, timeout=0.05)
        db.row_factory = sqlite3.Row
        return db

    def _initialize(self):
        with closing(self._connect()) as db, db:
            db.execute("BEGIN")
            db.execute("""CREATE TABLE IF NOT EXISTS runs (
                id TEXT PRIMARY KEY, started_at TEXT NOT NULL, ended_at TEXT,
                hump_count INTEGER, peak_mm INTEGER,
                end_reason TEXT, log_version INTEGER)""")
            columns = {row["name"] for row in db.execute("PRAGMA table_info(runs)")}
            if "end_reason" not in columns:
                db.execute("ALTER TABLE runs ADD COLUMN end_reason TEXT")
            if "log_version" not in columns:
                db.execute("ALTER TABLE runs ADD COLUMN log_version INTEGER")
            db.execute("""CREATE TABLE IF NOT EXISTS run_entries (
                sequence INTEGER PRIMARY KEY, run_id TEXT NOT NULL, buddy INTEGER NOT NULL,
                kind TEXT NOT NULL, received_at TEXT NOT NULL, data TEXT NOT NULL)""")
            db.execute("""CREATE INDEX IF NOT EXISTS run_entry_order
                ON run_entries (run_id, sequence)""")
            db.execute("""CREATE INDEX IF NOT EXISTS run_entry_stream
                ON run_entries (run_id, buddy, kind, sequence)""")
            db.execute("""CREATE TABLE IF NOT EXISTS run_entry_drops (
                run_id TEXT PRIMARY KEY, count INTEGER NOT NULL)""")
            rows = db.execute(f"""SELECT {_RUN_COLUMNS} FROM runs AS r
                ORDER BY r.started_at DESC, r.rowid DESC LIMIT ?""", (MAX_RUN_LOGS,)).fetchall()
            saved = {row["id"]: self._summary(row) for row in reversed(rows)}
            for run_id, record in saved.items():
                record["buddy_entries"] = self._buddy_counts(db, run_id) if record["has_details"] else {}
        saved.update(self.records)
        self.records = saved
        self._limit_visible_records()
        self._initialized = True

    def _summary(self, row):
        record = dict(row)
        record["has_details"] = bool(record["has_details"])
        # Label unfinished runs from a previous session without overwriting
        # their persisted metadata or supplying an unobserved end time.
        if (record.pop("log_version") == 1 and record["id"] not in self._open_runs
                and record["ended_at"] is None and record["end_reason"] is None):
            record["end_reason"] = "Dashboard restarted"
        return record

    def _limit_visible_records(self):
        while len(self.records) > MAX_RUN_LOGS:
            self.records.pop(next(iter(self.records)))

    @staticmethod
    def _buddy_counts(db, run_id):
        return {str(row["buddy"]): row["total"] for row in db.execute(
            "SELECT buddy, COUNT(*) AS total FROM run_entries WHERE run_id=? GROUP BY buddy",
            (run_id,),
        )}

    def record(self, run):
        run_id = run["id"]
        previous = self.records.get(run_id)
        record = dict(run, end_reason=run.get("end_reason"),
                      has_details=previous["has_details"] if previous else False,
                      buddy_entries=previous["buddy_entries"] if previous else {})
        if record["ended_at"] is None and record["end_reason"] is None:
            self._open_runs.add(run_id)
        else:
            self._open_runs.discard(run_id)
        self.records[run_id] = self._pending_runs[run_id] = record
        self._limit_visible_records()
        self._flush()

    def append(self, run_id, buddy, kind, received_at, data):
        record = self.records.get(run_id)
        if record is None:
            return
        payload = json.dumps(data, allow_nan=False, separators=(",", ":"))
        stream = (run_id, buddy, kind)
        waiting = self._pending_entries.setdefault(stream, deque(maxlen=MAX_ENTRIES_PER_STREAM))
        if len(waiting) == waiting.maxlen:
            self._pending_dropped[run_id] = self._pending_dropped.get(run_id, 0) + 1
        self._sequence += 1
        waiting.append((self._sequence, *stream, received_at, payload))
        record["has_details"] = True
        if record["ended_at"] is not None or self.clock() - self._last_checkpoint >= CHECKPOINT_S:
            self._flush()

    def _flush(self):
        have_pending = bool(self._pending_runs or self._pending_entries)
        if not have_pending and (self._initialized or not self.path.is_file()):
            return
        self._last_checkpoint = self.clock()
        try:
            if not self._initialized:
                self._initialize()
            if have_pending:
                self._write_pending()
            self.error = None
        except sqlite3.Error as exc:
            self.error = f"Could not save or load the run archive. {exc}"

    def _write_pending(self):
        drops = self._pending_dropped.copy()
        with closing(self._connect()) as db, db:
            db.executemany("""INSERT INTO runs
                (id, started_at, ended_at, hump_count, peak_mm, end_reason, log_version)
                VALUES (:id, :started_at, :ended_at, :hump_count, :peak_mm, :end_reason, 1)
                ON CONFLICT(id) DO UPDATE SET ended_at=excluded.ended_at,
                hump_count=excluded.hump_count, peak_mm=excluded.peak_mm,
                end_reason=excluded.end_reason""", self._pending_runs.values())
            entries = sorted(entry for stream in self._pending_entries.values() for entry in stream)
            db.executemany("""INSERT INTO run_entries (run_id, buddy, kind, received_at, data)
                VALUES (?, ?, ?, ?, ?)""", (entry[1:] for entry in entries))
            for stream in self._pending_entries:
                deleted = db.execute("""DELETE FROM run_entries
                    WHERE run_id=? AND buddy=? AND kind=? AND sequence < (
                        SELECT sequence FROM run_entries WHERE run_id=? AND buddy=? AND kind=?
                        ORDER BY sequence DESC LIMIT 1 OFFSET ?
                    )""", (*stream, *stream, MAX_ENTRIES_PER_STREAM - 1)).rowcount
                if deleted:
                    run_id = stream[0]
                    drops[run_id] = drops.get(run_id, 0) + deleted
            db.executemany("""INSERT INTO run_entry_drops (run_id, count) VALUES (?, ?)
                ON CONFLICT(run_id) DO UPDATE SET count=count+excluded.count""", drops.items())
            counts = {run_id: self._buddy_counts(db, run_id)
                      for run_id in {key[0] for key in self._pending_entries} if run_id in self.records}
        for run_id, buddy_entries in counts.items():
            self.records[run_id]["buddy_entries"] = buddy_entries
        self._pending_runs.clear()
        self._pending_entries.clear()
        self._pending_dropped.clear()

    def snapshot(self, current_id, active):
        self._flush()
        return dict(runs=[dict(run, active=active and run["id"] == current_id)
                          for run in reversed(self.records.values())],
                    current_id=current_id, error=self.error)

    def download(self, run_id, *, buddy=None):
        self._flush()
        if self.error:
            raise OSError(self.error)
        if not self._initialized:
            raise KeyError(run_id)
        try:
            with closing(self._connect()) as db:
                db.execute("BEGIN")
                row = db.execute(f"SELECT {_RUN_COLUMNS} FROM runs AS r WHERE r.id=?", (run_id,)).fetchone()
                if row is None:
                    raise KeyError(run_id)
                filters = "run_id=?" if buddy is None else "run_id=? AND buddy=?"
                params = (run_id,) if buddy is None else (run_id, buddy)
                entries = [dict(entry, data=json.loads(entry["data"])) for entry in db.execute(
                    f"""SELECT buddy, kind, received_at, data FROM run_entries
                        WHERE {filters} ORDER BY sequence""", params)]
                drops = db.execute("SELECT count FROM run_entry_drops WHERE run_id=?", (run_id,)).fetchone()
        except sqlite3.Error as exc:
            raise OSError(f"Could not read the run archive. {exc}") from exc
        return dict(schema_version=1, run=self._summary(row), entries=entries,
                    dropped_entries=drops["count"] if drops else 0)
