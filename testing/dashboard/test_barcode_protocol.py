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

# Golden vectors produced by the pre-v2 C encoder at main 46725c7. These must
# still decode while the robot is running its earlier firmware.
V1_PACKETS = [bytes.fromhex(value) for value in (
    "01f0ffffffc441000000000000004064",
    "01f0ffffff010500600000",
    "01f0ffffff0200880c0002000012585b5f2400400119080a04000000",
    "01f0ffffff03",
)]
V1_BOUNDARY = bytes.fromhex(
    "01f0ffffff0200f4fff5e5ffffffff5f5ffefffffffff5e5ffffffff"
    "5f5ffefffffffff5e5ffffffff5f5ffeffff1f")


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
        cls.batch_packets=[bytes.fromhex(line.split()[1]) for line in output.splitlines() if line.startswith("BCAP_BATCH_VECTOR ")]
        cls.tail=bytes.fromhex(next(line.split()[1] for line in output.splitlines() if line.startswith("BCAP_TAIL ")))

    def test_real_c_matches_independent_python_and_reconstructs_text(self):
        frames=[decode_barcode(p) for p in self.packets]
        self.assertEqual([f["kind"] for f in frames],["summary","episode","samples","end"])
        for frame,packet in zip(frames,self.packets):
            fields={k:v for k,v in frame.items() if k not in ("capture_id","kind","text")}
            self.assertEqual(encode_barcode(frame["capture_id"],frame["kind"],**fields),packet)
        self.assertEqual(frames[0]["capture_id"],0xfffffff0)
        frame=decode_barcode(self.boundary)
        self.assertEqual(len(self.boundary),164)
        self.assertEqual(frame["samples"],[tuple((4095,250,242,65535,255))]*24)
        with tempfile.TemporaryDirectory() as tmp:
            store=TraceStore(Path(tmp));state=DashboardState(store)
            for packet in self.packets:state.ingest("car/debug/barcode",packet)
            record=store.list()[0]
            self.assertTrue(record["complete"])
            self.assertEqual(record["sample_count"],3)
            self.assertEqual(store.get(record["id"])["samples"][1]["darkness"],1000)
            self.assertEqual(store.download(record["id"])[1],"\n".join(f["text"] for f in frames)+"\n")
            self.assertEqual(state.counters["malformed"],0)

    def test_existing_v1_captures_remain_readable_and_canonical(self):
        self.assertEqual(len(V1_BOUNDARY),47)
        self.assertEqual(decode_barcode(V1_BOUNDARY)["samples"],[(4095,250,242,65535,255)]*6)
        for old,new in zip(V1_PACKETS,self.packets):
            frame=decode_barcode(old)
            self.assertEqual(frame,decode_barcode(new))
            fields={k:v for k,v in frame.items() if k not in ("capture_id","kind","text")}
            self.assertEqual(encode_barcode(frame["capture_id"],frame["kind"],version=1,**fields),old)
        with tempfile.TemporaryDirectory() as tmp:
            store=TraceStore(Path(tmp));state=DashboardState(store)
            for packet in V1_PACKETS:state.ingest("car/debug/barcode",packet)
            self.assertTrue(store.list()[0]["complete"])
            self.assertEqual(store.list()[0]["sample_count"],3)
            self.assertEqual(state.counters["malformed"],0)

    def test_batched_c_packets_preserve_offsets_samples_and_episode_boundaries(self):
        frames=[decode_barcode(p) for p in self.batch_packets]
        samples=[f for f in frames if f["kind"]=="samples"]
        self.assertEqual([(f["episode"],f["index"],len(f["samples"])) for f in samples],
                         [(1,0,24),(1,24,24),(1,48,1),(2,0,24),(2,24,3)])
        self.assertEqual([row for f in samples for row in f["samples"]],
                         [(i,i,0,i,10) for i in range(76)])
        for frame,packet in zip(frames,self.batch_packets):
            fields={k:v for k,v in frame.items() if k not in ("capture_id","kind","text")}
            self.assertEqual(encode_barcode(frame["capture_id"],frame["kind"],**fields),packet)
        tail=decode_barcode(self.tail)
        self.assertEqual(tail["index"],2376)
        self.assertEqual(tail["samples"],[(i,0,0,i,0) for i in range(2376,2400)])
        with tempfile.TemporaryDirectory() as tmp:
            store=TraceStore(Path(tmp))
            for packet in self.batch_packets:
                store.ingest_binary(packet);store.ingest_binary(packet)
            record=store.list()[0]
            self.assertTrue(record["complete"])
            self.assertEqual(record["sample_count"],76)
        # Losing any batch remains visible, including either episode's short tail.
        for missing,frame in enumerate(frames):
            if frame["kind"]!="samples":continue
            with tempfile.TemporaryDirectory() as tmp:
                store=TraceStore(Path(tmp))
                for i,packet in enumerate(self.batch_packets):
                    if i!=missing:store.ingest_binary(packet)
                self.assertFalse(store.list()[0]["complete"])

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
        for payload in (b"[BCAP] end",b"",b"\x03"+self.packets[0][1:],
                        self.packets[0][:-1],self.packets[0]+b"\0",
                        self.packets[-1][:-1]+bytes([0xff])):
            with self.subTest(payload=payload),self.assertRaises(ValueError):decode_barcode(payload)
        for fields in (dict(episodes=13),dict(episodes=1,letters="a"),dict(episodes=1,letters="ABCDEFGHI")):
            with self.assertRaises(ValueError):encode_barcode(1,"summary",**fields)
        for row in ((4096,0,0,0,0),(0,251,0,0,0),(0,0,3,0,0),(0,0,20,0,0)):
            with self.assertRaises(ValueError):encode_barcode(1,"samples",episode=1,index=0,samples=[row])

    def test_versioned_sample_count_length_and_padding_limits(self):
        for version,limit,width in ((1,6,3),(2,24,5)):
            for count in (1,limit):
                packet=encode_barcode(0,"samples",version=version,episode=12,index=2400-count,
                                      samples=[(0,0,0,0,0)]*count)
                self.assertEqual(decode_barcode(packet)["samples"],[(0,0,0,0,0)]*count)
                with self.assertRaises(ValueError):decode_barcode(packet[:-1])
                with self.assertRaises(ValueError):decode_barcode(packet+b"\0")
                with self.assertRaises(ValueError):decode_barcode(packet[:-1]+bytes([packet[-1]|128]))
            for count,index in ((0,0),(limit+1,0),(limit,2401-limit),(1,-1)):
                with self.assertRaises(ValueError):
                    encode_barcode(1,"samples",version=version,episode=1,index=index,samples=[(0,0,0,0,0)]*count)
            # Count-minus-one lives at bit 58. Reserved count codes are rejected
            # before trying to read their nonexistent rows.
            for count_code in (limit,(1<<width)-1):
                bits=int.from_bytes(packet,"little")
                bits=(bits & ~(((1<<width)-1)<<58)) | count_code<<58
                with self.assertRaises(ValueError):decode_barcode(bits.to_bytes(len(packet),"little"))
        with self.assertRaises(ValueError):
            encode_barcode(1,"end",version=3)

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
