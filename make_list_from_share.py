#!/usr/bin/env python3
"""팀이 준 golden_v1_rtl_share 폴더에서 RTL 테스트벤치용 목록 파일을 만든다.

공유 폴더 구조
    golden_v1_rtl_share/expected/<id>/spec.mem, pre_env.mem, peak.mem, hist.mem, regs.mem

만드는 것
    list_spec_feat.txt : 한 줄 = "id fft spec pre_env peak hist regs" (절대경로)
    → tb_spec_feat_all.v 가 이 목록을 읽어 144개를 한 번에 검증한다.

공유 폴더 경로에 한글·띄어쓰기·괄호가 있으면 Vivado 시뮬레이터가 못 읽으므로,
--copy 를 주면 필요한 파일만 영문 경로로 복사한 뒤 그 경로로 목록을 만든다.

사용법:
    python make_list_from_share.py C:\\Users\\yjshi\\Downloads\\a\\golden_v1_rtl_share
    python make_list_from_share.py <공유폴더> --list C:\\Users\\yjshi\\Downloads\\a\\gm_vec\\list_spec_feat.txt
    python make_list_from_share.py <공유폴더> --copy C:\\Users\\yjshi\\Downloads\\a\\gm_vec
"""
import argparse
import shutil
from pathlib import Path

FILES = ['fft.mem', 'spec.mem', 'pre_env.mem', 'peak.mem', 'hist.mem', 'regs.mem']


def vivado_safe(path):
    s = Path(path).resolve().as_posix()
    return s.isascii() and not any(ch in s for ch in ' ()[]{}&')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('share', help='golden_v1_rtl_share 폴더')
    ap.add_argument('--list', default=None, help='만들 목록 파일 경로 (기본: 공유폴더 안 list_spec_feat.txt)')
    ap.add_argument('--copy', default=None, help='.mem 파일을 이 영문 경로 폴더로 복사한 뒤 목록 작성')
    args = ap.parse_args()

    share = Path(args.share)
    exp = share / 'expected'
    if not exp.is_dir():
        exp = share
    ids = sorted(d.name for d in exp.iterdir() if d.is_dir() and (d / 'regs.mem').is_file())
    if not ids:
        raise SystemExit(f'[중단] {exp} 안에 regs.mem 이 있는 샘플 폴더가 없습니다.')

    copy_dir = Path(args.copy) if args.copy else None
    if copy_dir is None and not vivado_safe(exp):
        raise SystemExit(f'[중단] 이 경로는 Vivado 가 못 읽습니다: {exp.resolve()}\n'
                         f'       --copy <영문 경로 폴더> 를 주면 필요한 파일만 복사해서 씁니다.')
    if copy_dir is not None:
        if not vivado_safe(copy_dir):
            raise SystemExit(f'[중단] --copy 경로에 한글·띄어쓰기·괄호가 있습니다: {copy_dir.resolve()}')
        copy_dir.mkdir(parents=True, exist_ok=True)

    list_path = Path(args.list) if args.list else (copy_dir or share) / 'list_spec_feat.txt'
    list_path.parent.mkdir(parents=True, exist_ok=True)
    if not vivado_safe(list_path.parent):
        raise SystemExit(f'[중단] 목록 파일 경로에 한글·띄어쓰기·괄호가 있습니다: {list_path.resolve()}')

    n_missing = 0
    with open(list_path, 'w', newline='\n') as lst:
        for sid in ids:
            paths = []
            ok = True
            for name in FILES:
                src = exp / sid / name
                if not src.is_file():
                    print(f'  [건너뜀] {sid}: {name} 없음')
                    ok = False
                    break
                if copy_dir is not None:
                    dst = copy_dir / f'{sid}_{name}'
                    shutil.copyfile(src, dst)
                    paths.append(dst.resolve().as_posix())
                else:
                    paths.append(src.resolve().as_posix())
            if ok:
                lst.write(sid + ' ' + ' '.join(paths) + '\n')
            else:
                n_missing += 1

    print(f'샘플 {len(ids) - n_missing}개 목록 작성' + (f' (파일 부족 {n_missing}개 제외)' if n_missing else ''))
    print(f'테스트벤치 LIST 경로: {list_path.resolve().as_posix()}')


if __name__ == '__main__':
    main()
