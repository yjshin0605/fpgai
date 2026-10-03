# 골든 모델 원본 ZIP

GitHub 파일 크기 제한 때문에 원본 ZIP을 24개로 분할했습니다. 아래 명령으로 원본을 복원할 수 있습니다.

```bash
python golden_archive/restore.py
```

Windows에서는 `py golden_archive/restore.py`를 사용하셔도 됩니다.
복원 스크립트는 파일 크기와 SHA-256을 확인한 후 `golden_archive/golden_v1_rtl_share.zip`을 생성합니다. 이후 ZIP을 압축 해제하여 사용합니다.
