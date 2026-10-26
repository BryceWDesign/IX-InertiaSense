#!/usr/bin/env python3
"""Verify SHA256 manifest for source distribution (not cryptographic signature)."""
from pathlib import Path
from hashlib import sha256
ROOT=Path(__file__).resolve().parents[1]
def main():
    manifest=ROOT/'MANIFEST.sha256'
    rows=[s.strip() for s in manifest.read_text().splitlines() if s.strip()]
    found=set()
    for row in rows:
        digest,name=row.split('  ',1)
        p=ROOT/name
        if not p.is_file() or sha256(p.read_bytes()).hexdigest()!=digest:
            raise SystemExit('FAIL: '+name)
        found.add(name)
    extras=[]
    for p in ROOT.rglob('*'):
        if p.is_file() and '.git' not in p.parts and 'build' not in p.parts and '__pycache__' not in p.parts and '.pytest_cache' not in p.parts:
            n=p.relative_to(ROOT).as_posix()
            if n not in found and n!='MANIFEST.sha256' and not n.endswith('.pyc') and n not in ('results/benchmark_ci.json','results/change_benchmark_ci.json'):extras.append(n)
    if extras:raise SystemExit('FAIL: unmanifested files: '+str(extras))
    print(f'PASS: {len(found)} files against MANIFEST.sha256')
if __name__=='__main__':main()
