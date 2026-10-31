#!/usr/bin/env python3
"""Download pinned source archives, verify their hashes and extract safely."""
import hashlib
import json
import shutil
import tarfile
import time
import urllib.request
from pathlib import Path

root = Path(__file__).resolve().parents[1]
cache = root / '.cache/downloads'
destination = root / '.agent_ws/src'
cache.mkdir(parents=True, exist_ok=True)
destination.mkdir(parents=True, exist_ok=True)
for item in json.loads((root / 'agent_sources.lock.json').read_text())['sources']:
    archive = cache / f"{item['name']}-{item['commit']}.tar.gz"
    for attempt in range(3):
        if archive.exists() and hashlib.sha256(archive.read_bytes()).hexdigest() == item['sha256']:
            break
        partial = archive.with_suffix('.part')
        try:
            with urllib.request.urlopen(item['url'], timeout=40) as response, partial.open('wb') as output:
                shutil.copyfileobj(response, output)
            if hashlib.sha256(partial.read_bytes()).hexdigest() != item['sha256']:
                raise RuntimeError(f"SHA-256 mismatch: {item['name']}")
            partial.replace(archive)
            break
        except Exception:
            partial.unlink(missing_ok=True)
            if attempt == 2:
                raise
            time.sleep(2)
    target = destination / item['name']
    marker = target / '.source-commit'
    if marker.exists() and marker.read_text().strip() == item['commit']:
        continue
    staging = destination / (item['name'] + '.extracting')
    if staging.exists():
        shutil.rmtree(staging)
    staging.mkdir()
    with tarfile.open(archive) as source:
        source.extractall(staging, filter='data')
    children = list(staging.iterdir())
    if len(children) != 1 or not children[0].is_dir():
        raise RuntimeError(f"Unexpected archive layout: {item['name']}")
    if target.exists():
        shutil.rmtree(target)
    children[0].rename(target)
    staging.rmdir()
    marker.write_text(item['commit'] + '\n')
    print(f"Verified {item['name']} {item['commit']}", flush=True)
