"""Check independent Code 39 fixtures against the supplied PDF (PyMuPDF)."""
import hashlib
import re
import sys
from pathlib import Path
import fitz

source = Path(sys.argv[1])
header = (Path(__file__).resolve().parents[1] / "testing/subsystem3/barcode_fixtures.h").read_text()
expected_hash = re.search(r"SHA256 ([a-f0-9]{64})", header).group(1)
assert hashlib.sha256(source.read_bytes()).hexdigest() == expected_hash, "PDF differs from fixture source"
with fitz.open(source) as document:
    assert len(document) == 4
    for index, page in enumerate(document, 1):
        pix = page.get_pixmap(matrix=fitz.Matrix(2, 2), colorspace=fitz.csGRAY, alpha=False)
        row = pix.samples[(pix.height // 5) * pix.stride: (pix.height // 5) * pix.stride + pix.width]
        black = [value < 128 for value in row]
        first = black.index(True)
        last = len(black) - 1 - black[::-1].index(True)
        widths, state, width = [], True, 0
        for colour in black[first:last + 1]:
            if colour != state:
                widths.append(width)
                state, width = colour, 0
            width += 1
        widths.append(width)
        match = re.search(r"g_pdf_page_" + str(index) + r"\[29\] = \{([^}]+)\}", header)
        expected = [int(value) for value in re.findall(r"(\d+)U", match.group(1))]
        assert widths == expected, f"Page {index}: extracted bars differ"
print("PASS: all four independent A/Z PDF fixtures match the supplied document")
