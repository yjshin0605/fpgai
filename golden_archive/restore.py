from pathlib import Path
import hashlib
import json

root = Path(__file__).resolve().parent
manifest = json.loads((root / "manifest.json").read_text())
data = b"".join((root / name).read_bytes() for name in manifest["parts"])
if len(data) != manifest["size_bytes"] or hashlib.sha256(data).hexdigest() != manifest["sha256"]:
    raise SystemExit("Archive checksum mismatch")
target = root / manifest["filename"]
target.write_bytes(data)
print(target)
