#!/usr/bin/env python3
"""dump_golden.py 출력(out/golden_v1/<id>/)을 RTL 테스트벤치가 읽을 수 있게 준비한다.

Vivado 시뮬레이터는 한글·띄어쓰기·괄호가 있는 경로를 못 읽으므로, 필요한 파일 5개를
영문 경로 폴더로 복사하고 list_spec_feat.txt 를 만든다.

  list_spec_feat.txt 한 줄 = "id spec.mem pre_env.mem peak.mem hist.mem regs.mem" (절대경로)

사용법:  python make_rtl_list.py <out/golden_v1 폴더> <복사할 영문 경로 폴더>
예:      python make_rtl_list.py out\\golden_v1 C:\\Users\\yjshi\\Downloads\\a\\gm_vec
"""
import shutil
import sys
from pathlib import Path

FILES = ['spec.mem', 'pre_env.mem', 'peak.mem', 'hist.mem', 'regs.mem']


def vivado_safe(path):
    s = path.resolve().as_posix()
    return s.isascii() and not any(ch in s for ch in ' ()[]{}&')


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    src, dst = Path(sys.argv[1]), Path(sys.argv[2])
    if not src.is_dir():
        sys.exit(f'[중단] 폴더가 없습니다: {src}')
    if not vivado_safe(dst):
        sys.exit(f'[중단] Vivado 가 못 읽는 경로입니다: {dst.resolve()}\n'
                 f'       영문·숫자만 있고 띄어쓰기·괄호가 없는 폴더를 주세요.')
    dst.mkdir(parents=True, exist_ok=True)

    ids = sorted(d.name for d in src.iterdir() if d.is_dir() and (d / 'regs.mem').is_file())
    if not ids:
        sys.exit(f'[중단] {src} 안에 regs.mem 이 있는 샘플 폴더가 없습니다. dump_golden.py 를 먼저 실행하세요.')

    with open(dst / 'list_spec_feat.txt', 'w', newline='\n') as lst:
        for sid in ids:
            paths = []
            for name in FILES:
                s = src / sid / name
                if not s.is_file():
                    sys.exit(f'[중단] 파일이 없습니다: {s}')
                d = dst / f'{sid}_{name}'
                shutil.copyfile(s, d)
                paths.append(d.resolve().as_posix())
            lst.write(sid + ' ' + ' '.join(paths) + '\n')

    print(f'{len(ids)}개 샘플 → {dst.resolve()}')
    print(f'테스트벤치 LIST 경로: {(dst / "list_spec_feat.txt").resolve().as_posix()}')


if __name__ == '__main__':
    main()
