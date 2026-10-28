from pathlib import Path
import hashlib,json,os,shutil
b=Path(__file__).resolve().parent
m=json.loads((b/'manifest.json').read_text())
t=Path(m['target'])
s=b/'nav2_params.yaml.before'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
assert sha(s)==m['before_sha256'], 'Backup checksum mismatch'
current=sha(t)
assert current in (m['before_sha256'],m['after_sha256']), 'Configuration has later edits; compare before restoring'
if current!=m['before_sha256']:
    tmp=t.with_name(t.name+'.height_fix_restore_tmp')
    assert not tmp.exists()
    shutil.copy2(s,tmp)
    os.replace(tmp,t)
assert sha(t)==m['before_sha256']
print('Restored and verified:',t)
print('Restart the navigation stack to load restored observation parameters.')
