#!/usr/bin/env python3
import pathlib,shutil,json,hashlib,uuid,os
backup=pathlib.Path(__file__).resolve().parent
manifest=json.loads((backup/'manifest.json').read_text())
for rel,expected in manifest.items():
    p=backup/rel
    if hashlib.sha256(p.read_bytes()).hexdigest()!=expected:
        raise SystemExit('Backup verification failed: '+rel)
items=json.loads((backup/'paths.json').read_text())
tag=uuid.uuid4().hex[:8]
retained=backup/('before_restore_'+tag)
retained.mkdir()
for label,target in items.items():
    dst=pathlib.Path(target)
    staged=dst.parent/(dst.name+'.restore-'+tag)
    shutil.copytree(backup/label,staged,symlinks=True)
    if dst.exists(): os.replace(dst,retained/label)
    os.replace(staged,dst)
print('Original source, install and build restored.')
print('Previous working copies retained at',retained)
print('Restart the affected ROS launch processes to use the restored executable.')
