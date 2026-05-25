# 5.23 After 변경 메모

## 목적

`docs/5_23_16:51커밋.md`는 5.23 당시 기록으로 유지한다.  
이 문서는 그 이후 협업 환경 정리를 위해 바뀐 내용만 적는다.

## 변경사항

### 1. OpenPCDet 경로를 팀원별 환경변수 기준으로 정리

- `app/core/config.py`에서 `OPENPCDET_ROOT` 환경변수를 우선 사용한다.
- 환경변수가 없으면 기본값으로 `/home/gh/workspaces/design_project/OpenPCDet`를 사용한다.
- OpenPCDet source tree는 ADAS repo에 포함하지 않는다.

팀원별 설정:

```bash
export OPENPCDET_ROOT=/path/to/OpenPCDet
```

### 2. 인지 모델 yaml 이름 정리

기존 sunny 환경 기준 파일명은 제거한다.

- 제거: `configs/openpcdet/dsvt_custom_ft.yaml`
- 제거: `configs/openpcdet/pointpillar_custom_ft.yaml`

대신 b4341 기준 custom yaml을 repo에 둔다.

- 추가: `configs/openpcdet/dsvt_pillar_sustech_ped_cyclist.yaml`
- 추가: `configs/openpcdet/pointpillar_sustech_ped_cyclist.yaml`

### 3. Checkpoint는 Git에서 제외

- `.gitignore`에 `app/perception/checkpoints/`를 추가했다.
- DSVT/PointPillars `.pth` 파일은 repo에 넣지 않는다.
- checkpoint는 팀 내에서 따로 공유하고 아래 위치에 복사한다.

```text
app/perception/checkpoints/dsvt_checkpoint_epoch_20.pth
app/perception/checkpoints/pointpillar_checkpoint_epoch_20.pth
```

### 4. OpenPCDet custom patch 추가

OpenPCDet 원본 clone 후 덮어쓸 최소 patch를 repo에 추가한다.

```text
external/openpcdet_patch/
  pcdet/datasets/__init__.py
  pcdet/datasets/custom/__init__.py
  pcdet/datasets/custom/custom_dataset.py
  tools/cfgs/dataset_configs/sustech_ped_cyclist_dataset.yaml
```

팀원 세팅 흐름:

```bash
git clone https://github.com/open-mmlab/OpenPCDet.git
cp -r ADAS_project/external/openpcdet_patch/* /path/to/OpenPCDet/
export OPENPCDET_ROOT=/path/to/OpenPCDet
mkdir -p ADAS_project/app/perception/checkpoints
```

그 후 checkpoint 두 개만 별도로 받아 `app/perception/checkpoints/`에 둔다.

## 현재 공유 정책

- Git에 포함: ADAS 코드, OpenPCDet custom yaml, OpenPCDet patch 파일
- 별도 공유: DSVT/PointPillars checkpoint `.pth`
- 각자 준비: OpenPCDet clone, CUDA/spconv 환경, `OPENPCDET_ROOT`
