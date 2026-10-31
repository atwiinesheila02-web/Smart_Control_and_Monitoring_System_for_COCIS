#!/usr/bin/env python3
"""Native logic tests with mocked ESP-IDF calls; does not build ESP32 firmware."""
from pathlib import Path
import subprocess,tempfile,json,os
root=Path(__file__).resolve().parents[1]
main=root/'main';tests=root/'tests'
sources=[p for p in sorted(main.glob('*.c')) if p.name not in {'access.c','safety.c','rooms.c'}]
flags=['gcc','-std=gnu11','-Wall','-Wextra','-Werror','-Wno-format','-Wno-format-truncation','-ffunction-sections','-fdata-sections','-I'+str(tests/'stubs'),'-I'+str(main)]
if os.environ.get('SMARTGUARD_SANITIZE') == '1':
    flags += ['-fsanitize=address,undefined','-fno-omit-frame-pointer']
# Syntax-check every translation unit, including normal standalone controls/storage.
subprocess.run(flags+['-fsyntax-only']+list(map(str,sorted(main.glob('*.c')))),check=True)
results=[]
with tempfile.TemporaryDirectory() as tmp:
    for test in ['reliability_test','emergency_test','passage_test','rfid_test']:
        binary=str(Path(tmp)/test)
        subprocess.run(flags+list(map(str,sources))+[str(tests/'mock_sdk.c'),str(tests/(test+'.c')),'-pthread','-lm','-Wl,--gc-sections','-o',binary],check=True)
        p=subprocess.run([binary],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,timeout=30,env={**os.environ,'ASAN_OPTIONS':'detect_leaks=0'})
        (tests/(test+'.log')).write_text(p.stdout+p.stderr)
        if p.returncode:
            print(p.stdout[-4000:]);print(p.stderr);raise SystemExit(p.returncode)
        results.append(p.stderr.strip())
        print(p.stderr.strip())
count=len(list(main.glob('*.c')))
print(f'PASS: all {count} application C files pass native syntax checks with SDK stubs')
validation_path=root/'validation.json'
previous=json.loads(validation_path.read_text(encoding='utf-8-sig')) if validation_path.exists() else {}
previous.update({'native_syntax_check':f'all {count} C files pass with mocked SDK declarations',
                 'test_results':results,
                 'native_sanitizers':'ASan/UBSan enabled and passed' if os.environ.get('SMARTGUARD_SANITIZE')=='1' else 'not enabled in this run'})
previous.setdefault('full_esp_idf_build','not run by native test runner')
previous.setdefault('wokwi_run','not run by native test runner')
import hashlib
previous['native_source_sha256']=hashlib.sha256(b''.join(p.read_bytes() for p in sorted(main.glob('*.[ch]')))).hexdigest()
validation_path.write_text(json.dumps(previous,indent=2)+'\n',encoding='utf-8')
