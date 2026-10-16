/* Independent fixtures from Barcode Sample.pdf; SHA256 4994bf35dead5fb06288bb4cd58672ce74dcca83a473c326f5e1aa968a694b74
 * PyMuPDF grayscale, 2x render, row height/5, threshold <128.
 * Widths exclude surrounding white margins. Tests add explicit quiet zones;
 * the PDF left margin itself is less than ten narrow modules. */
#ifndef BUDDY3_BARCODE_FIXTURES_H
#define BUDDY3_BARCODE_FIXTURES_H
static const uint32_t g_pdf_page_1[29] = {39U,119U,40U,40U,118U,40U,119U,40U,39U,40U,119U,39U,40U,40U,39U,119U,40U,39U,119U,40U,39U,119U,40U,40U,118U,40U,119U,40U,39U};
static const uint32_t g_pdf_page_2[29] = {20U,60U,20U,20U,59U,20U,60U,20U,20U,20U,59U,20U,20U,20U,20U,60U,20U,19U,60U,20U,20U,60U,20U,19U,60U,20U,60U,20U,20U};
static const uint32_t g_pdf_page_3[29] = {39U,119U,40U,40U,118U,40U,119U,40U,39U,40U,39U,119U,119U,40U,119U,39U,40U,40U,39U,40U,39U,119U,40U,40U,118U,40U,119U,40U,39U};
static const uint32_t g_pdf_page_4[29] = {20U,60U,20U,20U,59U,20U,60U,20U,20U,20U,20U,59U,60U,20U,59U,20U,20U,20U,20U,20U,20U,59U,20U,20U,60U,20U,60U,19U,20U};
#endif
