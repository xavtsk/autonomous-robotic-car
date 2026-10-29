"""Real C binary capture -> independent Python -> graphs and readable export."""
import io
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/"tools"))
from dashboard.barcode_protocol import decode_barcode, encode_barcode
from dashboard.traces import TraceStore
from dashboard.state import DashboardState
from buddy1_live import Folder, TraceFile, handle


class BarcodeWireTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        temp=tempfile.TemporaryDirectory();cls.addClassCleanup(temp.cleanup)
        binary=Path(temp.name)/"bcap_wire"
        subprocess.run(["cc","-std=c11","-Wall","-Wextra","-Werror",
                        "-I"+str(ROOT/"app_program/controller"),
                        str(ROOT/"testing/integration/bcap_wire_test.c"),
                        str(ROOT/"app_program/controller/barcode_capture.c"),"-o",str(binary)],check=True)
        output=subprocess.check_output([str(binary)],text=True)
        cls.packets=[bytes.fromhex(line.split()[1]) for line in output.splitlines() if line.startswith("BCAP_VECTOR ")]
        cls.boundary=bytes.fromhex(next(line.split()[1] for line in output.splitlines() if line.startswith("BCAP_BOUNDARY ")))

    def test_real_c_matches_independent_python_and_reconstructs_text(self):
        frames=[decode_barcode(p) for p in self.packets]
        self.assertEqual([f["kind"] for f in frames],["summary","episode","samples","end"])
        for frame,packet in zip(frames,self.packets):
            fields={k:v for k,v in frame.items() if k not in ("capture_id","kind","text")}
            self.assertEqual(encode_barcode(frame["capture_id"],frame["kind"],**fields),packet)
        self.assertEqual(frames[0]["capture_id"],0xfffffff0)
        frame=decode_barcode(self.boundary)
        self.assertEqual(len(self.boundary),47)
        self.assertEqual(frame["samples"],[tuple((4095,250,242,65535,255))]*6)
        with tempfile.TemporaryDirectory() as tmp:
            store=TraceStore(Path(tmp));state=DashboardState(store)
            for packet in self.packets:state.ingest("car/debug/barcode",packet)
            record=store.list()[0]
            self.assertTrue(record["complete"])
            self.assertEqual(record["sample_count"],3)
            self.assertEqual(store.get(record["id"])["samples"][1]["darkness"],1000)
            self.assertEqual(store.download(record["id"])[1],"\n".join(f["text"] for f in frames)+"\n")
            self.assertEqual(state.counters["malformed"],0)

    def test_missing_summary_header_samples_or_end_never_claim_complete(self):
        for missing in range(4):
            with tempfile.TemporaryDirectory() as tmp:
                store=TraceStore(Path(tmp))
                for i,p in enumerate(self.packets):
                    if i!=missing:store.ingest_binary(p)
                self.assertFalse(store.list()[0]["complete"])

    def test_duplicates_conflicts_interleaving_and_bounded_storage(self):
        with tempfile.TemporaryDirectory() as tmp:
            store=TraceStore(Path(tmp))
            for p in self.packets:
                store.ingest_binary(p);store.ingest_binary(p)
            for p in self.packets:store.ingest_binary(p)  # late retries
            self.assertEqual(len(store.list()),1)
            self.assertTrue(store.list()[0]["complete"])
            other=encode_barcode(123,"summary",episodes=0)
            store.ingest_binary(other);store.ingest_binary(encode_barcode(123,"end"))
            self.assertEqual(len(store.list()),2)
            self.assertTrue(all(r["complete"] for r in store.list()))
            bad=encode_barcode(0xfffffff0,"samples",episode=1,index=0,samples=[(99,0,1,0,9)])
            store.ingest_binary(bad)
            self.assertTrue(any(not r["complete"] for r in store.list()))
            for i in range(45):store.ingest_binary(encode_barcode(i,"summary",episodes=0))
            self.assertLessEqual(len(store.list()),40)
            self.assertLessEqual(len(store._binary),41)

    def test_hostile_packets_and_old_ascii_rejected(self):
        for payload in (b"[BCAP] end",b"",b"\x02"+self.packets[0][1:],
                        self.packets[0][:-1],self.packets[0]+b"\0",
                        self.packets[-1][:-1]+bytes([0xff])):
            with self.subTest(payload=payload),self.assertRaises(ValueError):decode_barcode(payload)
        for fields in (dict(episodes=13),dict(episodes=1,letters="a"),dict(episodes=1,letters="ABCDEFGHI")):
            with self.assertRaises(ValueError):encode_barcode(1,"summary",**fields)
        for row in ((4096,0,0,0,0),(0,251,0,0,0),(0,0,3,0,0),(0,0,20,0,0)):
            with self.assertRaises(ValueError):encode_barcode(1,"samples",episode=1,index=0,samples=[row])

    def test_terminal_saves_readable_text_and_deduplicates(self):
        with tempfile.TemporaryDirectory() as tmp:
            trace=TraceFile(tmp);out=io.StringIO()
            for packet in self.packets:
                handle("car/debug/barcode "+packet.hex(),Folder(True),out,trace)
                handle("car/debug/barcode "+packet.hex(),Folder(True),out,trace)
            files=list(Path(tmp).glob("*.txt"));self.assertEqual(len(files),1)
            self.assertIn("BC1 0: 100:0:1:0:9",files[0].read_text())
            self.assertEqual(files[0].read_text().count("run summary"),1)
            self.assertIn("barcode trace saved",out.getvalue())
