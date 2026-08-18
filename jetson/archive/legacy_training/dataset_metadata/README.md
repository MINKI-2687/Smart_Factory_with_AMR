# Rack ROI Dataset v1

3×3 적재함의 각 칸을 `OCCUPIED` 또는 `EMPTY`로 분류하기 위한 ROI 이미지 데이터셋이다.

## 구성

| split | 원본 프레임 | ROI 이미지 | OCCUPIED | EMPTY | 증강 |
|---|---:|---:|---:|---:|---|
| train | 40 | 1,800 | 915 | 885 | 원본당 4장 |
| validation | 10 | 90 | 45 | 45 | 없음 |
| test | 10 | 90 | 47 | 43 | 없음 |

이미지는 `images/<split>/<label>/` 아래에 있고, 학습용 클래스 이름은 `OCCUPIED`와 `EMPTY`이다.
칸마다 ROI 원본 크기가 조금 다르므로 학습 로더에서 모든 이미지를 동일한 크기(권장 `224×224`)로 `Resize`해야 한다.

## 분할 및 증강 원칙

- 캡처 프레임을 먼저 train/validation/test로 분리한 뒤 ROI를 생성했다.
- 동일 캡처 프레임은 둘 이상의 split에 존재하지 않는다.
- 증강은 train에만 적용했다.
- 증강 종류는 어둡게, 밝기+대비, 감마+대비+JPEG, 좌우 그림자+감마+JPEG이다.
- 공간 변환(회전, 반전, 이동, 자르기)은 적용하지 않았다.
- validation은 L00~L04 조명별 2프레임이며 모든 칸이 OCCUPIED 5회, EMPTY 5회다.

## 재현성과 주의점

- ROI 좌표는 `calv2`를 사용했다. 일부 원본 파일명에 남은 `calv1` 문자열은 과거 파일명일 뿐 실제 추출 설정을 뜻하지 않는다.
- `manifests/roi_manifest.csv`에 원본 프레임 경로, 원본/출력 SHA-256, 부모 샘플, 증강 파라미터가 기록되어 있다.
- `manifests/frame_manifest.csv`가 최종 프레임 분할의 기준이다. 원본 폴더명이나 glob만으로 split을 다시 만들면 안 된다.
- test는 s01/L00 동일 세션에서 떼어 둔 개발용 기준선이다. 실제 운용 성능을 주장하려면 별도 날짜·배치·조명으로 현장 test를 추가해야 한다.

## 무결성 검사

프로젝트 루트에서 다음 명령으로 다시 검사할 수 있다.

```bash
python -u tools/dataset/audit_prepared_dataset.py \
  --dataset-root . \
  --prepared-root prepared/rack_roi_dataset_v1 \
  --roi-config config/roi_calv2.json
```
