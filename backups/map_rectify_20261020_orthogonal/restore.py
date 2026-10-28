from pathlib import Path
import hashlib, os, shutil
root = Path(__file__).resolve().parent
for name, expected in [('rb3302_map.pgm', '9ab6ca26ec23103232584e5434f070fb8643b17e547775c4d30ec49dd74015a2'), ('rb3302_map.yaml', '1538025a82c8635d8885d3d0f7c560479e00187acd16927b221890cd9cba4c50')]:
    src = root / name
    assert hashlib.sha256(src.read_bytes()).hexdigest() == expected
    dst = Path('/home/ubuntu/rb3302_final_proj_code/student_ws/maps') / name
    tmp = dst.with_name(dst.name + '.restore.tmp')
    shutil.copy2(src, tmp)
    os.replace(tmp, dst)
    assert hashlib.sha256(dst.read_bytes()).hexdigest() == expected
print('Original PGM and YAML restored. Running nodes were not restarted.')
