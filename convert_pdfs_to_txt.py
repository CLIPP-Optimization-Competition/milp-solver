#!/usr/bin/env python3
"""Convert all PDF files in this repository to .txt files.

Each .txt file is placed alongside its source PDF with the same base name.
Requires PyMuPDF: pip install PyMuPDF
"""

import os
import glob
import fitz  # PyMuPDF

REPO_ROOT = os.path.dirname(os.path.abspath(__file__))


def pdf_to_txt(pdf_path: str) -> str:
    """Extract text from a PDF and write it to a .txt file. Returns the output path."""
    txt_path = os.path.splitext(pdf_path)[0] + ".txt"
    doc = fitz.open(pdf_path)
    text_parts = []
    for page in doc:
        text_parts.append(page.get_text())
    doc.close()

    with open(txt_path, "w", encoding="utf-8") as f:
        f.write("\n".join(text_parts))

    return txt_path


def main():
    pdf_files = sorted(glob.glob(os.path.join(REPO_ROOT, "**", "*.pdf"), recursive=True))
    print(f"Found {len(pdf_files)} PDF file(s).\n")

    success, failed = 0, 0
    for pdf in pdf_files:
        rel = os.path.relpath(pdf, REPO_ROOT)
        try:
            out = pdf_to_txt(pdf)
            print(f"  [OK] {rel}  ->  {os.path.relpath(out, REPO_ROOT)}")
            success += 1
        except Exception as e:
            print(f"  [FAIL] {rel}  ({e})")
            failed += 1

    print(f"\nDone. {success} converted, {failed} failed.")


if __name__ == "__main__":
    main()
