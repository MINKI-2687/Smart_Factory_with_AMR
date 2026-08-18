# %% [markdown]
# # 3×3 적재함 EMPTY/OCCUPIED 분류 — MobileNetV3-Small
#
# 이 노트북은 고정된 9개 ROI를 `EMPTY` 또는 `OCCUPIED`로 분류하는 기준 모델을 학습한다.
#
# 실행 전:
#
# 1. Colab 메뉴에서 **런타임 → 런타임 유형 변경 → T4 GPU**를 선택한다.
# 2. 런타임 버전은 **기본/최신**을 사용한다(과거 런타임 선택 금지).
# 3. 위에서 아래로 **모두 실행**한다.
# 4. Google Drive 마운트 승인 창에서 프로젝트 계정을 선택한다.
#
# 안전 원칙:
#
# - 제공된 train/validation/test 분할을 그대로 사용한다.
# - validation과 test에는 증강을 적용하지 않는다.
# - `OCCUPIED → EMPTY` 오판을 최우선으로 차단한다.
# - validation으로 모델과 임계값을 고정한 뒤 test를 한 번만 평가한다.
# - 안전 게이트가 실패하면 test 평가와 ONNX 기준 모델 확정을 중단한다.
# - TensorRT 엔진은 Colab이 아니라 Jetson에서 생성한다.

# %%
import subprocess
import sys

subprocess.check_call([
    sys.executable,
    "-m",
    "pip",
    "install",
    "-q",
    "onnx==1.22.0",
    "onnxscript==0.7.1",
    "onnxruntime==1.28.0",
])
print("PASS: ONNX dependencies installed")

# %%
import hashlib
import json
import os
import random
import re
import shutil
import stat
import subprocess
import sys
import time
import uuid
import zipfile
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path, PurePosixPath

os.environ["CUBLAS_WORKSPACE_CONFIG"] = ":4096:8"

import matplotlib.pyplot as plt
import numpy as np
import onnx
import onnxruntime as ort
import onnxscript
import pandas as pd
import PIL
import sklearn
import torch
import torch.nn as nn
import torchvision
from google.colab import drive
from sklearn.metrics import (
    accuracy_score,
    balanced_accuracy_score,
    confusion_matrix,
    precision_recall_fscore_support,
)
from torch.utils.data import DataLoader
from torchvision import datasets, transforms
from torchvision.models import (
    MobileNet_V3_Small_Weights,
    mobilenet_v3_small,
)
from torchvision.transforms import InterpolationMode

drive.mount("/content/drive")

DRIVE_ROOT = Path("/content/drive/MyDrive/ai/smart_logistics_robot_arm_2")
ZIP_PATH = DRIVE_ROOT / "rack_roi_dataset_v1_colab.zip"
NOTEBOOK_DIR = DRIVE_ROOT / "notebooks"
NOTEBOOK_SOURCE_PATH = (
    NOTEBOOK_DIR / "01_train_mobilenetv3_small_colab.py"
)
NOTEBOOK_IPYNB_PATH = (
    NOTEBOOK_DIR / "01_train_mobilenetv3_small_colab.ipynb"
)
NOTEBOOK_RELEASE_MANIFEST_PATH = (
    NOTEBOOK_DIR / "01_train_mobilenetv3_small_colab.release.json"
)
EXPECTED_ZIP_BYTES = 73_431_386
EXPECTED_ZIP_SHA256 = (
    "e86680610423f9817d1b7509cd451db1b4316c9a89083b4212eb08cee142aa74"
)
EXPECTED_TORCH = "2.11.0"
EXPECTED_TORCHVISION = "0.26.0"
EXPECTED_ONNX = "1.22.0"
EXPECTED_ONNXSCRIPT = "0.7.1"
EXPECTED_ONNXRUNTIME = "1.28.0"

WORK_ROOT = Path(f"/content/rack_training_{EXPECTED_ZIP_SHA256[:12]}")
EXTRACT_ROOT = WORK_ROOT / "dataset"
RUN_ID = (
    datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    + "_"
    + uuid.uuid4().hex[:8]
)
LOCAL_RUN_DIR = WORK_ROOT / "runs" / RUN_ID
DRIVE_CHECKPOINT_DIR = DRIVE_ROOT / "models" / "checkpoints" / RUN_ID
DRIVE_ONNX_DIR = DRIVE_ROOT / "models" / "onnx" / RUN_ID
DRIVE_REPORT_DIR = DRIVE_ROOT / "evaluation" / "reports" / RUN_ID

SEED = 20260801
INPUT_SIZE = 224
BATCH_SIZE = 64
NUM_WORKERS = 2
HEAD_EPOCHS = 4
FINETUNE_EPOCHS = 25
EARLY_STOPPING_PATIENCE = 6
MODEL_NAME = "mobilenet_v3_small"
POOLING_NAME = "spatial_mean_keepdim"
CLASS_TO_IDX = {"EMPTY": 0, "OCCUPIED": 1}
OCCUPIED_IDX = 1
IMAGENET_MEAN = [0.485, 0.456, 0.406]
IMAGENET_STD = [0.229, 0.224, 0.225]
CLUSTERED_SAMPLE_LIMITATION = (
    "Each 90-ROI split comes from only 10 captured frames; ROI outcomes "
    "within a frame are correlated, so no iid safety-error-rate claim is made."
)


def seed_everything(seed: int) -> None:
    os.environ["PYTHONHASHSEED"] = str(seed)
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)
    torch.backends.cudnn.benchmark = False
    torch.backends.cudnn.deterministic = True
    torch.use_deterministic_algorithms(True, warn_only=False)


seed_everything(SEED)

runtime_versions = {
    "python": sys.version.split()[0],
    "torch": str(torch.__version__),
    "torchvision": str(torchvision.__version__),
    "onnx": str(onnx.__version__),
    "onnxscript": str(onnxscript.__version__),
    "onnxruntime": str(ort.__version__),
    "numpy": str(np.__version__),
    "pandas": str(pd.__version__),
    "pillow": str(PIL.__version__),
    "scikit_learn": str(sklearn.__version__),
}
print(json.dumps(runtime_versions, indent=2))
print("CUDA available:", torch.cuda.is_available())
if torch.__version__.split("+")[0] != EXPECTED_TORCH:
    raise RuntimeError(
        f"Expected PyTorch {EXPECTED_TORCH}, got {torch.__version__}"
    )
if torchvision.__version__.split("+")[0] != EXPECTED_TORCHVISION:
    raise RuntimeError(
        f"Expected torchvision {EXPECTED_TORCHVISION}, got {torchvision.__version__}"
    )
if str(onnx.__version__) != EXPECTED_ONNX:
    raise RuntimeError(f"Expected ONNX {EXPECTED_ONNX}, got {onnx.__version__}")
if str(onnxscript.__version__) != EXPECTED_ONNXSCRIPT:
    raise RuntimeError(
        f"Expected ONNX Script {EXPECTED_ONNXSCRIPT}, got {onnxscript.__version__}"
    )
if str(ort.__version__) != EXPECTED_ONNXRUNTIME:
    raise RuntimeError(
        f"Expected ONNX Runtime {EXPECTED_ONNXRUNTIME}, got {ort.__version__}"
    )
if not torch.cuda.is_available():
    raise RuntimeError(
        "GPU가 선택되지 않았습니다. 런타임 유형을 T4 GPU로 바꾼 뒤 다시 실행하세요."
    )
DEVICE = torch.device("cuda")
GPU_NAME = torch.cuda.get_device_name(0)
print("GPU:", GPU_NAME)

# %%
def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def notebook_semantic_sha256(path: Path) -> str:
    notebook = json.loads(path.read_text(encoding="utf-8"))
    semantic_cells = []
    for cell in notebook.get("cells", []):
        source = cell.get("source", "")
        if isinstance(source, list):
            source = "".join(source)
        semantic_cells.append({
            "cell_type": cell.get("cell_type"),
            "source": source,
        })
    canonical = json.dumps(
        semantic_cells,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(canonical).hexdigest()


def validate_zip_members(archive: zipfile.ZipFile) -> None:
    for info in archive.infolist():
        member = PurePosixPath(info.filename)
        file_type = (info.external_attr >> 16) & 0o170000
        if member.is_absolute() or ".." in member.parts:
            raise RuntimeError(f"Unsafe ZIP path: {info.filename}")
        if file_type == stat.S_IFLNK:
            raise RuntimeError(f"ZIP symlink is forbidden: {info.filename}")


if not DRIVE_ROOT.is_dir():
    raise FileNotFoundError(f"Drive project root not found: {DRIVE_ROOT}")
if not ZIP_PATH.is_file():
    raise FileNotFoundError(f"Dataset ZIP not found: {ZIP_PATH}")
if ZIP_PATH.stat().st_size != EXPECTED_ZIP_BYTES:
    raise RuntimeError(
        f"ZIP size mismatch: {ZIP_PATH.stat().st_size} != {EXPECTED_ZIP_BYTES}"
    )

observed_zip_sha256 = sha256_file(ZIP_PATH)
print("ZIP SHA-256:", observed_zip_sha256)
if observed_zip_sha256 != EXPECTED_ZIP_SHA256:
    raise RuntimeError(
        "ZIP SHA-256 mismatch. 다른 파일을 학습하지 않도록 실행을 중단합니다."
    )

marker_path = EXTRACT_ROOT / ".extracted_zip_sha256"
if EXTRACT_ROOT.exists():
    if not marker_path.is_file():
        raise RuntimeError(
            f"기존 추출 폴더에 검증 marker가 없습니다: {EXTRACT_ROOT}"
        )
    if marker_path.read_text(encoding="utf-8").strip() != EXPECTED_ZIP_SHA256:
        raise RuntimeError("기존 추출 폴더가 현재 ZIP과 일치하지 않습니다.")
    print("Verified extraction reused:", EXTRACT_ROOT)
else:
    EXTRACT_ROOT.mkdir(parents=True, exist_ok=False)
    with zipfile.ZipFile(ZIP_PATH) as archive:
        validate_zip_members(archive)
        bad_member = archive.testzip()
        if bad_member is not None:
            raise RuntimeError(f"Corrupt ZIP member: {bad_member}")
        archive.extractall(EXTRACT_ROOT)
    marker_path.write_text(EXPECTED_ZIP_SHA256 + "\n", encoding="utf-8")
    print("ZIP extracted:", EXTRACT_ROOT)

audit_command = [
    sys.executable,
    "-u",
    str(EXTRACT_ROOT / "tools" / "audit_prepared_dataset.py"),
    "--dataset-root",
    str(EXTRACT_ROOT),
    "--prepared-root",
    str(EXTRACT_ROOT / "prepared" / "rack_roi_dataset_v1"),
    "--roi-config",
    str(EXTRACT_ROOT / "config" / "roi_calv2.json"),
]
subprocess.run(audit_command, check=True)
print("PASS: dataset archive and prepared dataset verified")

# %%
DATA_ROOT = EXTRACT_ROOT / "prepared" / "rack_roi_dataset_v1"
IMAGE_ROOT = DATA_ROOT / "images"
FRAME_MANIFEST_PATH = DATA_ROOT / "manifests" / "frame_manifest.csv"
ROI_MANIFEST_PATH = DATA_ROOT / "manifests" / "roi_manifest.csv"
ROI_CONFIG_PATH = EXTRACT_ROOT / "config" / "roi_calv2.json"

roi_manifest_table = pd.read_csv(ROI_MANIFEST_PATH, keep_default_na=False)
if roi_manifest_table["sample_id"].duplicated().any():
    raise RuntimeError("Duplicate sample_id in ROI manifest")
if roi_manifest_table["relative_path"].duplicated().any():
    raise RuntimeError("Duplicate relative_path in ROI manifest")

manifest_paths = set(roi_manifest_table["relative_path"])
actual_image_paths = {
    path.relative_to(DATA_ROOT).as_posix()
    for path in IMAGE_ROOT.rglob("*.png")
}
if manifest_paths != actual_image_paths:
    raise RuntimeError(
        "ROI manifest/image file set mismatch: "
        f"missing={len(manifest_paths - actual_image_paths)} "
        f"extra={len(actual_image_paths - manifest_paths)}"
    )

sample_lookup = roi_manifest_table.set_index("sample_id", drop=False)
for row in roi_manifest_table.itertuples(index=False):
    expected_path = (
        Path("images")
        / row.split
        / row.label
        / f"{row.sample_id}.png"
    ).as_posix()
    if row.relative_path != expected_path:
        raise RuntimeError(f"Label/path mismatch: {row.sample_id}")
    expected_parent_id = f"{row.capture_id}_{row.slot}_original"
    if row.variant == "original":
        if row.parent_sample_id:
            raise RuntimeError(f"Original has parent: {row.sample_id}")
        continue
    if row.parent_sample_id != expected_parent_id:
        raise RuntimeError(f"Wrong parent ID: {row.sample_id}")
    if expected_parent_id not in sample_lookup.index:
        raise RuntimeError(f"Missing parent sample: {row.sample_id}")
    parent = sample_lookup.loc[expected_parent_id]
    for field in (
        "capture_id",
        "slot",
        "split",
        "label",
        "source_frame_relpath",
        "source_frame_sha256",
    ):
        if getattr(row, field) != parent[field]:
            raise RuntimeError(
                f"Parent provenance mismatch {field}: {row.sample_id}"
            )
print("PASS: exact label paths and augmentation parent linkage")

LOCAL_RUN_DIR.mkdir(parents=True, exist_ok=False)
for output_dir in (
    DRIVE_CHECKPOINT_DIR,
    DRIVE_ONNX_DIR,
    DRIVE_REPORT_DIR,
):
    output_dir.mkdir(parents=True, exist_ok=False)

for required_notebook_file in (
    NOTEBOOK_SOURCE_PATH,
    NOTEBOOK_IPYNB_PATH,
    NOTEBOOK_RELEASE_MANIFEST_PATH,
):
    if not required_notebook_file.is_file():
        raise FileNotFoundError(
            f"Training code release file not found: {required_notebook_file}"
        )

release_manifest = json.loads(
    NOTEBOOK_RELEASE_MANIFEST_PATH.read_text(encoding="utf-8")
)
if release_manifest.get("schema_version") != 1:
    raise RuntimeError("Unsupported notebook release manifest schema")
release_files = release_manifest.get("files", {})
source_release = release_files.get("canonical_source", {})
notebook_release = release_files.get("colab_notebook", {})
if source_release.get("name") != NOTEBOOK_SOURCE_PATH.name:
    raise RuntimeError("Canonical source filename mismatch")
if notebook_release.get("name") != NOTEBOOK_IPYNB_PATH.name:
    raise RuntimeError("Colab notebook filename mismatch")

snapshot_source_path = LOCAL_RUN_DIR / "training_notebook_source.py"
snapshot_notebook_path = LOCAL_RUN_DIR / "executed_notebook_snapshot.ipynb"
snapshot_release_path = LOCAL_RUN_DIR / "notebook_release_manifest.json"
shutil.copy2(NOTEBOOK_SOURCE_PATH, snapshot_source_path)
shutil.copy2(NOTEBOOK_IPYNB_PATH, snapshot_notebook_path)
shutil.copy2(NOTEBOOK_RELEASE_MANIFEST_PATH, snapshot_release_path)

snapshot_source_sha256 = sha256_file(snapshot_source_path)
snapshot_notebook_semantic_sha256 = notebook_semantic_sha256(
    snapshot_notebook_path
)
if snapshot_source_sha256 != source_release.get("sha256"):
    raise RuntimeError(
        "Canonical notebook source SHA-256 mismatch; refusing to train"
    )
if (
    snapshot_notebook_semantic_sha256
    != notebook_release.get("semantic_sha256")
):
    raise RuntimeError(
        "Notebook code-cell SHA-256 mismatch; refusing to train"
    )

code_snapshot_payload = {
    "canonical_source_file": snapshot_source_path.name,
    "canonical_source_sha256": snapshot_source_sha256,
    "notebook_snapshot_file": snapshot_notebook_path.name,
    "notebook_snapshot_sha256": sha256_file(snapshot_notebook_path),
    "notebook_semantic_sha256": snapshot_notebook_semantic_sha256,
    "release_manifest_file": snapshot_release_path.name,
    "release_manifest_sha256": sha256_file(snapshot_release_path),
}
print("Run ID:", RUN_ID)
print("Checkpoint output:", DRIVE_CHECKPOINT_DIR)
print("ONNX output:", DRIVE_ONNX_DIR)
print("Report output:", DRIVE_REPORT_DIR)
print("Code snapshot:", json.dumps(code_snapshot_payload, indent=2))
print("PASS: exact training notebook code frozen")

image_transform = transforms.Compose([
    transforms.Resize(
        (INPUT_SIZE, INPUT_SIZE),
        interpolation=InterpolationMode.BILINEAR,
        antialias=True,
    ),
    transforms.ToTensor(),
    transforms.Normalize(mean=IMAGENET_MEAN, std=IMAGENET_STD),
])


class ImageFolderWithPath(datasets.ImageFolder):
    def __getitem__(self, index: int):
        image, label = super().__getitem__(index)
        path, _ = self.samples[index]
        return image, label, path


train_dataset = datasets.ImageFolder(
    IMAGE_ROOT / "train",
    transform=image_transform,
)
validation_dataset = ImageFolderWithPath(
    IMAGE_ROOT / "validation",
    transform=image_transform,
)
if train_dataset.class_to_idx != CLASS_TO_IDX:
    raise RuntimeError(f"Train class mapping mismatch: {train_dataset.class_to_idx}")
if validation_dataset.class_to_idx != CLASS_TO_IDX:
    raise RuntimeError(
        f"Validation class mapping mismatch: {validation_dataset.class_to_idx}"
    )
if (len(train_dataset), len(validation_dataset)) != (1800, 90):
    raise RuntimeError("Unexpected train/validation sizes")


def dataset_class_counts(dataset) -> dict[str, int]:
    counts = Counter(dataset.classes[label] for _, label in dataset.samples)
    return dict(sorted(counts.items()))


print("class_to_idx:", train_dataset.class_to_idx)
print("train:", len(train_dataset), dataset_class_counts(train_dataset))
print(
    "validation:",
    len(validation_dataset),
    dataset_class_counts(validation_dataset),
)


def seed_worker(worker_id: int) -> None:
    worker_seed = (SEED + worker_id) % (2**32)
    np.random.seed(worker_seed)
    random.seed(worker_seed)


train_generator = torch.Generator()
train_generator.manual_seed(SEED)
loader_kwargs = {
    "batch_size": BATCH_SIZE,
    "num_workers": NUM_WORKERS,
    "pin_memory": True,
    "persistent_workers": NUM_WORKERS > 0,
    "worker_init_fn": seed_worker,
}
train_loader = DataLoader(
    train_dataset,
    shuffle=True,
    generator=train_generator,
    **loader_kwargs,
)
validation_loader = DataLoader(
    validation_dataset,
    shuffle=False,
    **loader_kwargs,
)

frame_manifest = pd.read_csv(FRAME_MANIFEST_PATH)
frame_lookup = frame_manifest.set_index("capture_id")
manifest_hashes = {
    "frame_manifest_sha256": sha256_file(FRAME_MANIFEST_PATH),
    "roi_manifest_sha256": sha256_file(ROI_MANIFEST_PATH),
}
print("Manifest hashes:", manifest_hashes)
print("PASS: loaders and frame metadata ready")

# %%
images, labels, paths = next(iter(validation_loader))
mean_tensor = torch.tensor(IMAGENET_MEAN).view(3, 1, 1)
std_tensor = torch.tensor(IMAGENET_STD).view(3, 1, 1)
figure, axes = plt.subplots(3, 3, figsize=(12, 9))
for axis, image, label, path_text in zip(
    axes.ravel(),
    images[:9],
    labels[:9],
    paths[:9],
):
    display_image = (image.cpu() * std_tensor + mean_tensor).clamp(0, 1)
    axis.imshow(display_image.permute(1, 2, 0))
    axis.set_title(
        f"{validation_dataset.classes[int(label)]}\n{Path(path_text).name}"
    )
    axis.axis("off")
figure.suptitle("Validation ROI sanity check")
figure.tight_layout()
plt.show()

# %%
class SpatialMeanPool(nn.Module):
    def forward(self, features: torch.Tensor) -> torch.Tensor:
        return features.mean(dim=(-2, -1), keepdim=True)


weights = MobileNet_V3_Small_Weights.DEFAULT
model = mobilenet_v3_small(weights=weights)
if not isinstance(model.avgpool, nn.AdaptiveAvgPool2d):
    raise RuntimeError(f"Unexpected pooling module: {type(model.avgpool)}")
model.avgpool = SpatialMeanPool()
old_head = model.classifier[-1]
if not isinstance(old_head, nn.Linear):
    raise RuntimeError(f"Unexpected classifier head: {type(old_head)}")
model.classifier[-1] = nn.Linear(old_head.in_features, len(CLASS_TO_IDX))
model = model.to(DEVICE)
criterion = nn.CrossEntropyLoss()
scaler = torch.amp.GradScaler("cuda", enabled=True)

smoke_images, smoke_labels = next(iter(train_loader))
smoke_images = smoke_images[:4].to(DEVICE)
smoke_labels = smoke_labels[:4].to(DEVICE)
model.eval()
smoke_logits = model(smoke_images)
smoke_loss = criterion(smoke_logits, smoke_labels)
smoke_loss.backward()
smoke_gradients = [
    parameter.grad
    for parameter in model.parameters()
    if parameter.requires_grad and parameter.grad is not None
]
if smoke_logits.shape != (4, 2):
    raise RuntimeError(f"Unexpected model output: {smoke_logits.shape}")
if not smoke_gradients:
    raise RuntimeError("Full-graph smoke test produced no gradients")
if not torch.isfinite(smoke_loss) or not all(
    torch.isfinite(gradient).all() for gradient in smoke_gradients
):
    raise RuntimeError("Non-finite smoke-test loss or gradient")
model.zero_grad(set_to_none=True)
train_generator.manual_seed(SEED)
print("PASS: deterministic full-graph forward/backward smoke test")

history: list[dict[str, object]] = []
best = {
    "val_loss": float("inf"),
    "epoch": None,
    "stage": None,
}
LOCAL_BEST_PATH = LOCAL_RUN_DIR / "best.pt"
LOCAL_LAST_PATH = LOCAL_RUN_DIR / "last.pt"


def cpu_state_dict(module: nn.Module) -> dict[str, torch.Tensor]:
    return {
        key: value.detach().cpu()
        for key, value in module.state_dict().items()
    }


def compute_binary_metrics(
    labels_array: np.ndarray,
    occupied_probabilities: np.ndarray,
    threshold: float = 0.5,
) -> dict[str, float | int]:
    predictions = (occupied_probabilities >= threshold).astype(np.int64)
    tn, fp, fn, tp = confusion_matrix(
        labels_array,
        predictions,
        labels=[0, 1],
    ).ravel()
    precision, recall, f1, _ = precision_recall_fscore_support(
        labels_array,
        predictions,
        labels=[1],
        average=None,
        zero_division=0,
    )
    return {
        "accuracy": float(accuracy_score(labels_array, predictions)),
        "balanced_accuracy": float(
            balanced_accuracy_score(labels_array, predictions)
        ),
        "occupied_precision": float(precision[0]),
        "occupied_recall": float(recall[0]),
        "occupied_f1": float(f1[0]),
        "true_empty": int(tn),
        "false_occupied": int(fp),
        "dangerous_false_empty": int(fn),
        "true_occupied": int(tp),
    }


@torch.inference_mode()
def evaluate_validation():
    model.eval()
    total_loss = 0.0
    total_items = 0
    all_labels = []
    all_probabilities = []
    for batch_images, batch_labels, _ in validation_loader:
        batch_images = batch_images.to(DEVICE, non_blocking=True)
        batch_labels = batch_labels.to(DEVICE, non_blocking=True)
        with torch.amp.autocast(
            device_type="cuda",
            dtype=torch.float16,
            enabled=True,
        ):
            logits = model(batch_images)
            loss = criterion(logits, batch_labels)
        if not torch.isfinite(loss) or not torch.isfinite(logits).all():
            raise RuntimeError("Non-finite validation logits or loss")
        total_loss += float(loss.item()) * batch_labels.size(0)
        total_items += batch_labels.size(0)
        all_labels.append(batch_labels.cpu().numpy())
        all_probabilities.append(
            torch.softmax(logits.float(), dim=1)[:, OCCUPIED_IDX].cpu().numpy()
        )
    return (
        total_loss / total_items,
        np.concatenate(all_labels),
        np.concatenate(all_probabilities),
    )


def train_one_epoch(optimizer, feature_extractor_frozen: bool) -> float:
    model.train()
    if feature_extractor_frozen:
        model.features.eval()
    total_loss = 0.0
    total_items = 0
    for batch_images, batch_labels in train_loader:
        batch_images = batch_images.to(DEVICE, non_blocking=True)
        batch_labels = batch_labels.to(DEVICE, non_blocking=True)
        optimizer.zero_grad(set_to_none=True)
        with torch.amp.autocast(
            device_type="cuda",
            dtype=torch.float16,
            enabled=True,
        ):
            logits = model(batch_images)
            loss = criterion(logits, batch_labels)
        if not torch.isfinite(loss) or not torch.isfinite(logits).all():
            raise RuntimeError("Non-finite training logits or loss")
        scaler.scale(loss).backward()
        scaler.step(optimizer)
        scaler.update()
        total_loss += float(loss.item()) * batch_labels.size(0)
        total_items += batch_labels.size(0)
    return total_loss / total_items


def checkpoint_payload(epoch: int, stage: str, val_loss: float) -> dict:
    return {
        "format_version": 1,
        "model_name": MODEL_NAME,
        "pooling": POOLING_NAME,
        "model_state_dict": cpu_state_dict(model),
        "epoch": int(epoch),
        "stage": stage,
        "best_val_loss": float(val_loss),
        "class_to_idx": CLASS_TO_IDX,
        "occupied_index": OCCUPIED_IDX,
        "input_size": INPUT_SIZE,
        "source_color_space": "RGB",
        "imagenet_mean": IMAGENET_MEAN,
        "imagenet_std": IMAGENET_STD,
        "dataset_zip_sha256": EXPECTED_ZIP_SHA256,
        **manifest_hashes,
        "seed": SEED,
        "runtime_versions": runtime_versions,
        "gpu": GPU_NAME,
    }


def save_checkpoint(path: Path, epoch: int, stage: str, val_loss: float) -> None:
    torch.save(checkpoint_payload(epoch, stage, val_loss), path)


def fit_stage(
    stage_name: str,
    epochs: int,
    optimizer,
    scheduler,
    feature_extractor_frozen: bool,
    patience: int | None,
) -> None:
    epochs_without_improvement = 0
    for stage_epoch in range(1, epochs + 1):
        global_epoch = len(history) + 1
        start_time = time.perf_counter()
        train_loss = train_one_epoch(
            optimizer,
            feature_extractor_frozen,
        )
        val_loss, val_labels, val_probabilities = evaluate_validation()
        metrics = compute_binary_metrics(val_labels, val_probabilities)
        row = {
            "epoch": global_epoch,
            "stage": stage_name,
            "stage_epoch": stage_epoch,
            "train_loss": train_loss,
            "val_loss": val_loss,
            "learning_rate": optimizer.param_groups[0]["lr"],
            **metrics,
        }
        history.append(row)

        improved = val_loss < best["val_loss"] - 1e-12
        if improved:
            best.update({
                "val_loss": val_loss,
                "epoch": global_epoch,
                "stage": stage_name,
            })
            save_checkpoint(
                LOCAL_BEST_PATH,
                global_epoch,
                stage_name,
                val_loss,
            )
            epochs_without_improvement = 0
        else:
            epochs_without_improvement += 1

        scheduler.step()
        elapsed = time.perf_counter() - start_time
        print(
            f"[{stage_name}] epoch={global_epoch:02d} "
            f"train_loss={train_loss:.4f} val_loss={val_loss:.4f} "
            f"bal_acc={metrics['balanced_accuracy']:.4f} "
            f"occ_recall={metrics['occupied_recall']:.4f} "
            f"danger_FN={metrics['dangerous_false_empty']} "
            f"best={improved} time={elapsed:.1f}s"
        )

        if patience is not None and epochs_without_improvement >= patience:
            print(f"Early stopping: {stage_name}")
            break

# %%
for parameter in model.features.parameters():
    parameter.requires_grad = False
head_optimizer = torch.optim.AdamW(
    model.classifier.parameters(),
    lr=1e-3,
    weight_decay=1e-4,
)
head_scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
    head_optimizer,
    T_max=HEAD_EPOCHS,
)
fit_stage(
    stage_name="classifier_head",
    epochs=HEAD_EPOCHS,
    optimizer=head_optimizer,
    scheduler=head_scheduler,
    feature_extractor_frozen=True,
    patience=None,
)

for parameter in model.features.parameters():
    parameter.requires_grad = True
finetune_optimizer = torch.optim.AdamW(
    [
        {"params": model.features.parameters(), "lr": 1e-4},
        {"params": model.classifier.parameters(), "lr": 3e-4},
    ],
    weight_decay=1e-4,
)
finetune_scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
    finetune_optimizer,
    T_max=FINETUNE_EPOCHS,
)
fit_stage(
    stage_name="full_finetune",
    epochs=FINETUNE_EPOCHS,
    optimizer=finetune_optimizer,
    scheduler=finetune_scheduler,
    feature_extractor_frozen=False,
    patience=EARLY_STOPPING_PATIENCE,
)

if not LOCAL_BEST_PATH.is_file():
    raise RuntimeError("Best checkpoint was not created")
if (DRIVE_CHECKPOINT_DIR / "best.pt").exists():
    raise RuntimeError("Refusing to overwrite best.pt")
shutil.copy2(LOCAL_BEST_PATH, DRIVE_CHECKPOINT_DIR / "best.pt")
save_checkpoint(
    LOCAL_LAST_PATH,
    len(history),
    "training_complete",
    float(history[-1]["val_loss"]),
)
shutil.copy2(LOCAL_LAST_PATH, DRIVE_CHECKPOINT_DIR / "last.pt")

checkpoint = torch.load(
    LOCAL_BEST_PATH,
    map_location="cpu",
    weights_only=True,
)
model.load_state_dict(checkpoint["model_state_dict"], strict=True)
model = model.to("cpu").eval()
pd.DataFrame(history).to_csv(
    LOCAL_RUN_DIR / "history.csv",
    index=False,
)
print(
    "PASS: best checkpoint loaded",
    checkpoint["epoch"],
    checkpoint["stage"],
    checkpoint["best_val_loss"],
)

# %%
class OccupiedProbability(nn.Module):
    def __init__(self, base_model: nn.Module):
        super().__init__()
        self.base_model = base_model

    def forward(self, images: torch.Tensor) -> torch.Tensor:
        logits = self.base_model(images)
        return torch.softmax(logits, dim=1)[:, OCCUPIED_IDX : OCCUPIED_IDX + 1]


export_model = OccupiedProbability(model).eval()
LOCAL_ONNX_PATH = LOCAL_RUN_DIR / "rack_occupancy_mobilenetv3_small.onnx"
dummy_input = torch.zeros(
    1,
    3,
    INPUT_SIZE,
    INPUT_SIZE,
    dtype=torch.float32,
)
batch_dimension = torch.export.Dim("batch", min=1, max=9)
onnx_program = torch.onnx.export(
    export_model,
    (dummy_input,),
    input_names=["images"],
    output_names=["p_occupied"],
    opset_version=17,
    dynamo=True,
    verify=True,
    dynamic_shapes=({0: batch_dimension},),
)
onnx_program.save(str(LOCAL_ONNX_PATH), external_data=False)
onnx.checker.check_model(str(LOCAL_ONNX_PATH), full_check=True)
print("ONNX SHA-256:", sha256_file(LOCAL_ONNX_PATH))
print("PASS: ONNX export and checker")

# %%
sample_pattern = re.compile(r"^(a\d+)_(C[123]_L[123])_")
validation_parity_loader = DataLoader(
    validation_dataset,
    batch_size=9,
    shuffle=False,
    num_workers=NUM_WORKERS,
    pin_memory=False,
    persistent_workers=NUM_WORKERS > 0,
    worker_init_fn=seed_worker,
)
ort_session = ort.InferenceSession(
    str(LOCAL_ONNX_PATH),
    providers=["CPUExecutionProvider"],
)
if ort_session.get_inputs()[0].name != "images":
    raise RuntimeError("Unexpected ONNX input name")
if ort_session.get_inputs()[0].type != "tensor(float)":
    raise RuntimeError("Unexpected ONNX input dtype")
if ort_session.get_outputs()[0].name != "p_occupied":
    raise RuntimeError("Unexpected ONNX output name")


def assert_valid_probabilities(name: str, values: np.ndarray) -> None:
    array = np.asarray(values)
    if not np.isfinite(array).all():
        bad_count = int(np.size(array) - np.isfinite(array).sum())
        raise RuntimeError(f"{name} contains {bad_count} NaN/Inf values")
    if np.any(array < 0.0) or np.any(array > 1.0):
        raise RuntimeError(
            f"{name} is outside [0, 1]: min={array.min()} max={array.max()}"
        )


@torch.inference_mode()
def collect_pytorch_onnx_predictions(loader) -> pd.DataFrame:
    export_model.eval()
    rows = []
    for batch_images, batch_labels, batch_paths in loader:
        contiguous_images = batch_images.float().cpu().contiguous()
        pytorch_probabilities = (
            export_model(contiguous_images).squeeze(1).cpu().numpy()
        )
        onnx_probabilities = ort_session.run(
            ["p_occupied"],
            {"images": np.ascontiguousarray(
                contiguous_images.numpy(),
                dtype=np.float32,
            )},
        )[0].squeeze(1)
        assert_valid_probabilities(
            "validation PyTorch probabilities",
            pytorch_probabilities,
        )
        assert_valid_probabilities(
            "validation ONNX probabilities",
            onnx_probabilities,
        )
        for label, p_torch, p_onnx, path_text in zip(
            batch_labels.numpy(),
            pytorch_probabilities,
            onnx_probabilities,
            batch_paths,
        ):
            path = Path(path_text)
            match = sample_pattern.match(path.stem)
            if match is None:
                raise RuntimeError(f"Unexpected sample name: {path.name}")
            capture_id, slot = match.groups()
            frame_row = frame_lookup.loc[capture_id]
            rows.append({
                "relative_path": path.relative_to(DATA_ROOT).as_posix(),
                "capture_id": capture_id,
                "slot": slot,
                "captured_light": frame_row["captured_light"],
                "label_index": int(label),
                "label": validation_dataset.classes[int(label)],
                "p_occupied_pytorch": float(p_torch),
                "p_occupied": float(p_onnx),
                "backend_abs_difference": float(abs(p_torch - p_onnx)),
            })
    return pd.DataFrame(rows)


validation_predictions = collect_pytorch_onnx_predictions(
    validation_parity_loader
)
if len(validation_predictions) != 90:
    raise RuntimeError("Unexpected validation prediction count")

np.testing.assert_allclose(
    validation_predictions["p_occupied"].to_numpy(),
    validation_predictions["p_occupied_pytorch"].to_numpy(),
    rtol=1e-4,
    atol=1e-4,
)
max_backend_abs_difference = float(
    validation_predictions["backend_abs_difference"].max()
)
BACKEND_EPSILON = max(0.02, max_backend_abs_difference + 0.005)

first_image = validation_dataset[0][0].unsqueeze(0).numpy()
first_nine = torch.stack(
    [validation_dataset[index][0] for index in range(9)]
).numpy()
if ort_session.run(["p_occupied"], {"images": first_image})[0].shape != (1, 1):
    raise RuntimeError("ONNX batch=1 output shape mismatch")
if ort_session.run(["p_occupied"], {"images": first_nine})[0].shape != (9, 1):
    raise RuntimeError("ONNX batch=9 output shape mismatch")

validation_binary_metrics = compute_binary_metrics(
    validation_predictions["label_index"].to_numpy(),
    validation_predictions["p_occupied"].to_numpy(),
)
print("Validation ONNX @ 0.5")
print(json.dumps(validation_binary_metrics, indent=2))
print("max |p_pytorch - p_onnx|:", max_backend_abs_difference)
print("safety epsilon:", BACKEND_EPSILON)
print("PASS: full-validation PyTorch/ONNX Runtime parity")

# %%
def triage_decisions(
    probabilities: np.ndarray,
    t_empty: float,
    t_occupied: float,
) -> np.ndarray:
    if not 0.0 <= t_empty < t_occupied <= 1.0:
        raise ValueError("Threshold order is invalid")
    return np.where(
        probabilities <= t_empty,
        "EMPTY",
        np.where(
            probabilities >= t_occupied,
            "OCCUPIED",
            "UNKNOWN",
        ),
    )


def summarize_triage_arrays(
    labels_array: np.ndarray,
    decisions: np.ndarray,
) -> dict[str, float | int]:
    actual_empty = labels_array == CLASS_TO_IDX["EMPTY"]
    actual_occupied = labels_array == CLASS_TO_IDX["OCCUPIED"]
    empty_to_empty = int(np.sum(actual_empty & (decisions == "EMPTY")))
    empty_to_unknown = int(np.sum(actual_empty & (decisions == "UNKNOWN")))
    empty_to_occupied = int(np.sum(actual_empty & (decisions == "OCCUPIED")))
    occupied_to_occupied = int(
        np.sum(actual_occupied & (decisions == "OCCUPIED"))
    )
    occupied_to_unknown = int(
        np.sum(actual_occupied & (decisions == "UNKNOWN"))
    )
    occupied_to_empty = int(np.sum(actual_occupied & (decisions == "EMPTY")))
    known_count = empty_to_empty + empty_to_occupied + occupied_to_occupied + occupied_to_empty
    correct_known = empty_to_empty + occupied_to_occupied
    n_empty = int(np.sum(actual_empty))
    n_occupied = int(np.sum(actual_occupied))
    total = len(labels_array)
    return {
        "n": int(total),
        "n_empty": n_empty,
        "n_occupied": n_occupied,
        "empty_to_empty": empty_to_empty,
        "empty_to_unknown": empty_to_unknown,
        "empty_to_occupied": empty_to_occupied,
        "occupied_to_occupied": occupied_to_occupied,
        "occupied_to_unknown": occupied_to_unknown,
        "occupied_to_empty": occupied_to_empty,
        "coverage": float(known_count / total),
        "known_accuracy": float(correct_known / known_count)
        if known_count
        else 0.0,
        "empty_action_recall": float(empty_to_empty / n_empty)
        if n_empty
        else 0.0,
        "occupied_action_recall": float(occupied_to_occupied / n_occupied)
        if n_occupied
        else 0.0,
    }


def grouped_triage_summary(
    frame: pd.DataFrame,
    group_column: str,
) -> pd.DataFrame:
    rows = []
    for group_value, group in frame.groupby(group_column, sort=True):
        rows.append({
            group_column: group_value,
            **summarize_triage_arrays(
                group["label_index"].to_numpy(),
                group["decision"].to_numpy(),
            ),
        })
    return pd.DataFrame(rows)


def frame_triage_summary(
    frame: pd.DataFrame,
) -> tuple[pd.DataFrame, dict[str, int]]:
    rows = []
    for capture_id, group in frame.groupby("capture_id", sort=True):
        if len(group) != 9 or group["slot"].nunique() != 9:
            raise RuntimeError(
                f"Capture {capture_id} does not contain exactly 9 unique slots"
            )
        expected = np.where(
            group["label_index"].to_numpy() == CLASS_TO_IDX["OCCUPIED"],
            "OCCUPIED",
            "EMPTY",
        )
        decisions = group["decision"].to_numpy()
        actual_occupied = (
            group["label_index"].to_numpy() == CLASS_TO_IDX["OCCUPIED"]
        )
        unsafe = bool(np.any(actual_occupied & (decisions == "EMPTY")))
        has_cross_error = bool(np.any(
            ((expected == "EMPTY") & (decisions == "OCCUPIED"))
            | ((expected == "OCCUPIED") & (decisions == "EMPTY"))
        ))
        has_unknown = bool(np.any(decisions == "UNKNOWN"))
        fully_correct = bool(np.array_equal(expected, decisions))
        rows.append({
            "capture_id": capture_id,
            "roi_count": int(len(group)),
            "unsafe_occupied_to_empty": unsafe,
            "has_cross_error": has_cross_error,
            "has_unknown": has_unknown,
            "fully_correct": fully_correct,
        })
    details = pd.DataFrame(rows)
    summary = {
        "frames": int(len(details)),
        "unsafe_frames": int(details["unsafe_occupied_to_empty"].sum()),
        "cross_error_frames": int(details["has_cross_error"].sum()),
        "frames_with_unknown": int(details["has_unknown"].sum()),
        "fully_correct_frames": int(details["fully_correct"].sum()),
    }
    return details, summary


def threshold_candidate_metrics(
    labels_array: np.ndarray,
    probabilities: np.ndarray,
    t_empty: float,
    t_occupied: float,
    epsilon: float,
) -> dict[str, float | int]:
    decisions = triage_decisions(probabilities, t_empty, t_occupied)
    summary = summarize_triage_arrays(labels_array, decisions)
    actual_empty = labels_array == CLASS_TO_IDX["EMPTY"]
    actual_occupied = labels_array == CLASS_TO_IDX["OCCUPIED"]
    robust_occupied_to_empty = int(
        np.sum(actual_occupied & ((probabilities - epsilon) <= t_empty))
    )
    robust_empty_to_occupied = int(
        np.sum(actual_empty & ((probabilities + epsilon) >= t_occupied))
    )
    separation_margin = float(min(
        np.min(probabilities[actual_occupied] - epsilon - t_empty),
        np.min(t_occupied - (probabilities[actual_empty] + epsilon)),
    ))
    return {
        **summary,
        "robust_occupied_to_empty": robust_occupied_to_empty,
        "robust_empty_to_occupied": robust_empty_to_occupied,
        "min_class_action_recall": min(
            summary["empty_action_recall"],
            summary["occupied_action_recall"],
        ),
        "separation_margin": separation_margin,
    }


def validation_group_gate(
    frame: pd.DataFrame,
    decisions: np.ndarray,
) -> tuple[bool, pd.DataFrame, pd.DataFrame]:
    evaluated = frame.copy()
    evaluated["decision"] = decisions
    by_slot = grouped_triage_summary(evaluated, "slot")
    by_light = grouped_triage_summary(evaluated, "captured_light")
    slot_ok = (
        (by_slot["occupied_to_empty"] == 0)
        & (by_slot["empty_to_occupied"] == 0)
        & (by_slot["coverage"] >= 0.70)
        & (by_slot["empty_to_empty"] >= 3)
        & (by_slot["occupied_to_occupied"] >= 3)
    ).all()
    light_ok = (
        (by_light["occupied_to_empty"] == 0)
        & (by_light["empty_to_occupied"] == 0)
        & (by_light["coverage"] >= (12 / 18))
        & (by_light["empty_to_empty"] >= 6)
        & (by_light["occupied_to_occupied"] >= 6)
    ).all()
    return bool(slot_ok and light_ok), by_slot, by_light


labels_array = validation_predictions["label_index"].to_numpy()
probabilities = validation_predictions["p_occupied"].to_numpy()
search_rows = []
for empty_step in range(0, 200):
    t_empty = empty_step / 200.0
    for occupied_step in range(empty_step + 1, 201):
        t_occupied = occupied_step / 200.0
        metrics = threshold_candidate_metrics(
            labels_array,
            probabilities,
            t_empty,
            t_occupied,
            BACKEND_EPSILON,
        )
        search_rows.append({
            "t_empty": t_empty,
            "t_occupied": t_occupied,
            "band_width": t_occupied - t_empty,
            **metrics,
        })

threshold_search = pd.DataFrame(search_rows)
threshold_search.to_csv(
    LOCAL_RUN_DIR / "validation_threshold_search.csv",
    index=False,
)
eligible = threshold_search[
    (threshold_search["robust_occupied_to_empty"] == 0)
    & (threshold_search["robust_empty_to_occupied"] == 0)
    & (threshold_search["occupied_to_empty"] == 0)
    & (threshold_search["empty_to_occupied"] == 0)
    & (threshold_search["known_accuracy"] == 1.0)
    & (threshold_search["coverage"] >= 0.80)
].copy()
eligible = eligible.sort_values(
    [
        "min_class_action_recall",
        "coverage",
        "separation_margin",
        "band_width",
        "t_empty",
        "t_occupied",
    ],
    ascending=[False, False, False, True, True, False],
    kind="mergesort",
)

selected_row = None
validation_by_slot = None
validation_by_light = None
for _, candidate in eligible.iterrows():
    candidate_decisions = triage_decisions(
        probabilities,
        float(candidate["t_empty"]),
        float(candidate["t_occupied"]),
    )
    group_ok, by_slot, by_light = validation_group_gate(
        validation_predictions,
        candidate_decisions,
    )
    if group_ok:
        selected_row = candidate
        validation_by_slot = by_slot
        validation_by_light = by_light
        break

if selected_row is None:
    diagnostics = threshold_search.sort_values(
        [
            "robust_occupied_to_empty",
            "robust_empty_to_occupied",
            "min_class_action_recall",
            "coverage",
        ],
        ascending=[True, True, False, False],
    ).head(30)
    diagnostics.to_csv(
        DRIVE_REPORT_DIR / "threshold_failure_diagnostics.csv",
        index=False,
    )
    display(diagnostics)
    raise RuntimeError(
        "STOP: validation threshold safety gate failed. "
        "Test와 기준 ONNX 확정을 진행하지 않습니다."
    )

T_EMPTY = float(selected_row["t_empty"])
T_OCCUPIED = float(selected_row["t_occupied"])
validation_predictions["decision"] = triage_decisions(
    probabilities,
    T_EMPTY,
    T_OCCUPIED,
)
validation_triage_metrics = summarize_triage_arrays(
    labels_array,
    validation_predictions["decision"].to_numpy(),
)
validation_by_frame, validation_frame_metrics = frame_triage_summary(
    validation_predictions
)
if validation_frame_metrics["unsafe_frames"] != 0:
    raise RuntimeError("STOP: validation contains an unsafe frame")
validation_predictions.to_csv(
    LOCAL_RUN_DIR / "validation_predictions.csv",
    index=False,
)
validation_by_slot.to_csv(
    LOCAL_RUN_DIR / "validation_by_slot.csv",
    index=False,
)
validation_by_light.to_csv(
    LOCAL_RUN_DIR / "validation_by_light.csv",
    index=False,
)
validation_by_frame.to_csv(
    LOCAL_RUN_DIR / "validation_by_frame.csv",
    index=False,
)
print("T_EMPTY:", T_EMPTY)
print("T_OCCUPIED:", T_OCCUPIED)
print(json.dumps(validation_triage_metrics, indent=2))
print("Validation frames:", json.dumps(validation_frame_metrics, indent=2))
display(validation_by_slot)
display(validation_by_light)
print("PASS: validation safety gate")

# %%
def write_json(path: Path, payload: dict) -> None:
    if path.exists():
        raise RuntimeError(f"Refusing to overwrite: {path}")
    path.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def write_text_if_identical(path: Path, text_content: str) -> None:
    if path.exists():
        if path.read_text(encoding="utf-8") != text_content:
            raise RuntimeError(
                f"Refusing to replace different existing file: {path}"
            )
        return
    path.write_text(text_content, encoding="utf-8")


def write_json_if_identical(path: Path, payload: dict) -> None:
    write_text_if_identical(
        path,
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
    )


def copy_without_overwrite(source_path: Path, destination_path: Path) -> None:
    if destination_path.exists():
        if sha256_file(source_path) != sha256_file(destination_path):
            raise RuntimeError(
                f"Refusing to overwrite different file: {destination_path}"
            )
        return
    shutil.copy2(source_path, destination_path)


def build_raw_test_identity(
    source_frame_hashes,
    expected_frame_count: int = 10,
) -> tuple[list[str], str]:
    unique_hashes = sorted(set(str(value) for value in source_frame_hashes))
    if len(unique_hashes) != expected_frame_count:
        raise RuntimeError(
            "Unexpected unique raw source-frame count: "
            f"{len(unique_hashes)} != {expected_frame_count}"
        )
    if any(not re.fullmatch(r"[0-9a-f]{64}", value) for value in unique_hashes):
        raise RuntimeError("Invalid raw source-frame SHA-256")
    identity_payload = {
        "raw_test_source_frame_sha256": unique_hashes,
    }
    fingerprint = hashlib.sha256(
        json.dumps(
            identity_payload,
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
    ).hexdigest()
    return unique_hashes, fingerprint


np.testing.assert_array_equal(
    triage_decisions(
        validation_predictions["p_occupied_pytorch"].to_numpy(),
        T_EMPTY,
        T_OCCUPIED,
    ),
    validation_predictions["decision"].to_numpy(),
)

figure, axis = plt.subplots(figsize=(8, 5))
for label_name, color in (("EMPTY", "tab:blue"), ("OCCUPIED", "tab:orange")):
    values = validation_predictions.loc[
        validation_predictions["label"] == label_name,
        "p_occupied",
    ]
    axis.hist(values, bins=20, alpha=0.65, label=label_name, color=color)
axis.axvline(T_EMPTY, color="green", linestyle="--", label="T_EMPTY")
axis.axvline(T_OCCUPIED, color="red", linestyle="--", label="T_OCCUPIED")
axis.set_xlabel("ONNX p_occupied")
axis.set_ylabel("Count")
axis.set_title("Validation probability distribution")
axis.legend()
figure.tight_layout()
figure.savefig(
    LOCAL_RUN_DIR / "validation_probability_distribution.png",
    dpi=160,
)
plt.show()

checkpoint_sha256 = sha256_file(LOCAL_BEST_PATH)
onnx_sha256 = sha256_file(LOCAL_ONNX_PATH)
preprocessing_payload = {
    "schema_version": 1,
    "input_name": "images",
    "output_name": "p_occupied",
    "input_shape": ["N", 3, INPUT_SIZE, INPUT_SIZE],
    "input_dtype": "float32",
    "source_color_space": "RGB",
    "tensor_layout": "NCHW",
    "resize": {
        "height": INPUT_SIZE,
        "width": INPUT_SIZE,
        "interpolation": "bilinear",
        "antialias": True,
        "center_crop": False,
    },
    "uint8_to_float": "value / 255.0",
    "normalization": {
        "mean": IMAGENET_MEAN,
        "std": IMAGENET_STD,
    },
    "class_to_index": CLASS_TO_IDX,
    "occupied_index": OCCUPIED_IDX,
}
training_config_payload = {
    "schema_version": 1,
    "run_id": RUN_ID,
    "seed": SEED,
    "model_name": MODEL_NAME,
    "pooling": POOLING_NAME,
    "dataset_zip_sha256": EXPECTED_ZIP_SHA256,
    **manifest_hashes,
    "batch_size": BATCH_SIZE,
    "head_epochs": HEAD_EPOCHS,
    "finetune_epochs_max": FINETUNE_EPOCHS,
    "early_stopping_patience": EARLY_STOPPING_PATIENCE,
    "optimizer": "AdamW",
    "loss": "CrossEntropyLoss",
    "runtime_random_augmentation": False,
    "checkpoint_selection": "minimum validation loss",
    "test_used_for_selection": False,
    "code_snapshot": code_snapshot_payload,
}
environment_payload = {
    "schema_version": 1,
    "run_id": RUN_ID,
    "versions": runtime_versions,
    "cuda_runtime": torch.version.cuda,
    "cudnn_version": torch.backends.cudnn.version(),
    "gpu": GPU_NAME,
    "determinism": {
        "cublas_workspace_config": os.environ.get(
            "CUBLAS_WORKSPACE_CONFIG"
        ),
        "deterministic_algorithms": (
            torch.are_deterministic_algorithms_enabled()
        ),
        "deterministic_debug_mode": int(
            torch.get_deterministic_debug_mode()
        ),
        "cudnn_benchmark": bool(torch.backends.cudnn.benchmark),
        "cudnn_deterministic": bool(torch.backends.cudnn.deterministic),
    },
}
onnx_parity_payload = {
    "schema_version": 1,
    "validation_sample_count": int(len(validation_predictions)),
    "rtol": 1e-4,
    "atol": 1e-4,
    "max_abs_probability_difference": max_backend_abs_difference,
    "batch_1_output_shape": [1, 1],
    "batch_9_output_shape": [9, 1],
    "triage_decisions_identical": True,
}
validation_metrics_payload = {
    "schema_version": 1,
    "binary_at_0p5": validation_binary_metrics,
    "triage": validation_triage_metrics,
    "by_slot": validation_by_slot.to_dict(orient="records"),
    "by_light": validation_by_light.to_dict(orient="records"),
    "by_frame": validation_frame_metrics,
    "statistical_limitation": CLUSTERED_SAMPLE_LIMITATION,
    "meaning": "development validation gate, not a field safety claim",
}
thresholds_payload = {
    "schema_version": 1,
    "model": {
        "architecture": MODEL_NAME,
        "pooling": POOLING_NAME,
        "checkpoint_file": "best.pt",
        "checkpoint_sha256": checkpoint_sha256,
        "onnx_file": LOCAL_ONNX_PATH.name,
        "onnx_sha256": onnx_sha256,
        "class_to_index": CLASS_TO_IDX,
        "score_definition": "ONNX output p_occupied",
    },
    "data": {
        "dataset_zip_sha256": EXPECTED_ZIP_SHA256,
        **manifest_hashes,
        "validation_capture_ids": sorted(
            validation_predictions["capture_id"].unique().tolist()
        ),
    },
    "code": code_snapshot_payload,
    "selection": {
        "source": "validation_original_only",
        "search_resolution": 0.005,
        "backend_max_abs_difference": max_backend_abs_difference,
        "epsilon_floor": 0.02,
        "epsilon_total": BACKEND_EPSILON,
        "objective": (
            "zero robust cross-errors, then maximum balanced action coverage"
        ),
    },
    "thresholds": {
        "t_empty": T_EMPTY,
        "t_occupied": T_OCCUPIED,
        "empty_rule": "p_occupied <= t_empty",
        "occupied_rule": "p_occupied >= t_occupied",
        "unknown_rule": "t_empty < p_occupied < t_occupied",
    },
    "validation_metrics": validation_triage_metrics,
    "validation_frame_metrics": validation_frame_metrics,
    "statistical_limitation": CLUSTERED_SAMPLE_LIMITATION,
    "test_policy": {
        "consumed_at_threshold_freeze": False,
        "retuning_from_test_forbidden": True,
    },
}
roi_config = json.loads(ROI_CONFIG_PATH.read_text(encoding="utf-8"))
model_metadata_payload = {
    "schema_version": 1,
    "status": "VALIDATION_GATE_PASSED_TEST_NOT_OPENED",
    "run_id": RUN_ID,
    "preprocessing": preprocessing_payload,
    "thresholds": thresholds_payload["thresholds"],
    "backend_epsilon": BACKEND_EPSILON,
    "code_snapshot": code_snapshot_payload,
    "roi_calibration": roi_config,
    "temporal_filter_initial_policy": {
        "window_frames": 5,
        "required_agreement_frames": 4,
        "status": "not_validated_yet",
    },
    "test_limitation": (
        "s01/L00 same-site development baseline; "
        "not a final field-performance claim"
    ),
}

write_json(LOCAL_RUN_DIR / "preprocessing.json", preprocessing_payload)
write_json(LOCAL_RUN_DIR / "training_config.json", training_config_payload)
write_json(LOCAL_RUN_DIR / "environment.json", environment_payload)
reproducibility_artifact_hashes = {
    "preprocessing_sha256": sha256_file(
        LOCAL_RUN_DIR / "preprocessing.json"
    ),
    "training_config_sha256": sha256_file(
        LOCAL_RUN_DIR / "training_config.json"
    ),
    "environment_sha256": sha256_file(
        LOCAL_RUN_DIR / "environment.json"
    ),
}
thresholds_payload["reproducibility_artifacts"] = (
    reproducibility_artifact_hashes
)
model_metadata_payload["reproducibility_artifacts"] = (
    reproducibility_artifact_hashes
)
write_json(LOCAL_RUN_DIR / "onnx_parity.json", onnx_parity_payload)
write_json(
    LOCAL_RUN_DIR / "validation_metrics.json",
    validation_metrics_payload,
)
write_json(LOCAL_RUN_DIR / "thresholds.json", thresholds_payload)
write_json(LOCAL_RUN_DIR / "model_metadata.json", model_metadata_payload)

validation_artifacts = [
    "history.csv",
    "validation_predictions.csv",
    "validation_by_slot.csv",
    "validation_by_light.csv",
    "validation_by_frame.csv",
    "validation_threshold_search.csv",
    "validation_probability_distribution.png",
    "preprocessing.json",
    "training_config.json",
    "environment.json",
    "onnx_parity.json",
    "validation_metrics.json",
    "thresholds.json",
    "model_metadata.json",
    "training_notebook_source.py",
    "executed_notebook_snapshot.ipynb",
    "notebook_release_manifest.json",
]
for file_name in validation_artifacts:
    copy_without_overwrite(
        LOCAL_RUN_DIR / file_name,
        DRIVE_REPORT_DIR / file_name,
    )

thresholds_sha256 = sha256_file(LOCAL_RUN_DIR / "thresholds.json")
test_rows_for_fingerprint = roi_manifest_table[
    roi_manifest_table["split"] == "test"
].sort_values("sample_id")
test_source_frame_sha256, TEST_SET_FINGERPRINT = build_raw_test_identity(
    test_rows_for_fingerprint["source_frame_sha256"].tolist()
)
TEST_LOCK_DIR = DRIVE_ROOT / "evaluation" / "test_locks"
SOURCE_FRAME_LOCK_DIR = TEST_LOCK_DIR / "source_frames"
TEST_LOCK_DIR.mkdir(parents=True, exist_ok=True)
SOURCE_FRAME_LOCK_DIR.mkdir(parents=True, exist_ok=True)
TEST_LOCK_PATH = TEST_LOCK_DIR / f"{TEST_SET_FINGERPRINT}.lock.json"
SOURCE_FRAME_LOCK_PATHS = [
    SOURCE_FRAME_LOCK_DIR / f"{source_sha256}.lock.json"
    for source_sha256 in test_source_frame_sha256
]
existing_test_locks = [
    path for path in [TEST_LOCK_PATH, *SOURCE_FRAME_LOCK_PATHS]
    if path.exists()
]
if existing_test_locks:
    raise RuntimeError(
        "STOP: at least one raw test frame was already consumed. "
        f"Existing lock: {existing_test_locks[0]}"
    )
test_consumption_payload = {
    "schema_version": 1,
    "run_id": RUN_ID,
    "started_at_utc": datetime.now(timezone.utc).isoformat(),
    "test_set_fingerprint": TEST_SET_FINGERPRINT,
    "raw_test_source_frame_sha256": test_source_frame_sha256,
    "dataset_zip_sha256": EXPECTED_ZIP_SHA256,
    **manifest_hashes,
    "thresholds_sha256": thresholds_sha256,
    "checkpoint_sha256": checkpoint_sha256,
    "onnx_sha256": onnx_sha256,
    "policy": (
        "Test is consumed by this run. Do not tune model or thresholds from it."
    ),
}
try:
    with TEST_LOCK_PATH.open("x", encoding="utf-8") as lock_handle:
        json.dump(
            test_consumption_payload,
            lock_handle,
            indent=2,
            sort_keys=True,
        )
        lock_handle.write("\n")
except FileExistsError as exc:
    raise RuntimeError(
        "STOP: this exact test set was already consumed. "
        f"Existing lock: {TEST_LOCK_PATH}"
    ) from exc
for source_sha256, source_lock_path in zip(
    test_source_frame_sha256,
    SOURCE_FRAME_LOCK_PATHS,
):
    source_lock_payload = {
        "schema_version": 1,
        "run_id": RUN_ID,
        "started_at_utc": test_consumption_payload["started_at_utc"],
        "test_set_fingerprint": TEST_SET_FINGERPRINT,
        "raw_source_frame_sha256": source_sha256,
        "policy": "This raw test frame is consumed and must not be reused.",
    }
    try:
        with source_lock_path.open("x", encoding="utf-8") as lock_handle:
            json.dump(
                source_lock_payload,
                lock_handle,
                indent=2,
                sort_keys=True,
            )
            lock_handle.write("\n")
    except FileExistsError as exc:
        raise RuntimeError(
            "STOP: a raw test frame was concurrently consumed. "
            f"Existing lock: {source_lock_path}"
        ) from exc
test_lock_evidence = {
    "test_consumed": True,
    "test_set_fingerprint": TEST_SET_FINGERPRINT,
    "set_lock": {
        "relative_path": TEST_LOCK_PATH.relative_to(DRIVE_ROOT).as_posix(),
        "sha256": sha256_file(TEST_LOCK_PATH),
    },
    "source_frame_locks": [
        {
            "raw_source_frame_sha256": source_sha256,
            "relative_path": lock_path.relative_to(DRIVE_ROOT).as_posix(),
            "sha256": sha256_file(lock_path),
        }
        for source_sha256, lock_path in zip(
            test_source_frame_sha256,
            SOURCE_FRAME_LOCK_PATHS,
        )
    ],
}
write_json(
    DRIVE_REPORT_DIR / "test_evaluation_started.json",
    {
        **test_consumption_payload,
        "lock_evidence": test_lock_evidence,
    },
)
TEST_EVALUATION_EXECUTED = False
print("PASS: validation artifacts frozen before test")

# %%
if TEST_EVALUATION_EXECUTED:
    raise RuntimeError("STOP: test evaluation already executed in this runtime")
TEST_EVALUATION_EXECUTED = True
observed_test_lock = json.loads(TEST_LOCK_PATH.read_text(encoding="utf-8"))
if observed_test_lock.get("run_id") != RUN_ID:
    raise RuntimeError("Test lock does not belong to this run")
if sha256_file(TEST_LOCK_PATH) != test_lock_evidence["set_lock"]["sha256"]:
    raise RuntimeError("Test-set lock SHA-256 changed before evaluation")
for source_lock_path, source_lock_record in zip(
    SOURCE_FRAME_LOCK_PATHS,
    test_lock_evidence["source_frame_locks"],
):
    observed_source_lock = json.loads(
        source_lock_path.read_text(encoding="utf-8")
    )
    if observed_source_lock.get("run_id") != RUN_ID:
        raise RuntimeError(
            f"Source-frame lock does not belong to this run: {source_lock_path}"
        )
    if sha256_file(source_lock_path) != source_lock_record["sha256"]:
        raise RuntimeError(
            f"Source-frame lock SHA-256 changed: {source_lock_path}"
        )

test_dataset = ImageFolderWithPath(
    IMAGE_ROOT / "test",
    transform=image_transform,
)
if test_dataset.class_to_idx != CLASS_TO_IDX:
    raise RuntimeError(f"Test class mapping mismatch: {test_dataset.class_to_idx}")
if len(test_dataset) != 90:
    raise RuntimeError(f"Unexpected test size: {len(test_dataset)}")
test_loader = DataLoader(
    test_dataset,
    batch_size=9,
    shuffle=False,
    num_workers=NUM_WORKERS,
    pin_memory=False,
    persistent_workers=NUM_WORKERS > 0,
    worker_init_fn=seed_worker,
)


def collect_onnx_predictions(loader, dataset) -> pd.DataFrame:
    rows = []
    for batch_images, batch_labels, batch_paths in loader:
        contiguous_images = np.ascontiguousarray(
            batch_images.float().cpu().numpy(),
            dtype=np.float32,
        )
        probabilities = ort_session.run(
            ["p_occupied"],
            {"images": contiguous_images},
        )[0].squeeze(1)
        assert_valid_probabilities("test ONNX probabilities", probabilities)
        for label, probability, path_text in zip(
            batch_labels.numpy(),
            probabilities,
            batch_paths,
        ):
            path = Path(path_text)
            match = sample_pattern.match(path.stem)
            if match is None:
                raise RuntimeError(f"Unexpected sample name: {path.name}")
            capture_id, slot = match.groups()
            frame_row = frame_lookup.loc[capture_id]
            rows.append({
                "relative_path": path.relative_to(DATA_ROOT).as_posix(),
                "capture_id": capture_id,
                "slot": slot,
                "captured_light": frame_row["captured_light"],
                "label_index": int(label),
                "label": dataset.classes[int(label)],
                "p_occupied": float(probability),
            })
    return pd.DataFrame(rows)


test_predictions = collect_onnx_predictions(test_loader, test_dataset)
test_predictions["decision"] = triage_decisions(
    test_predictions["p_occupied"].to_numpy(),
    T_EMPTY,
    T_OCCUPIED,
)
test_binary_metrics = compute_binary_metrics(
    test_predictions["label_index"].to_numpy(),
    test_predictions["p_occupied"].to_numpy(),
)
test_triage_metrics = summarize_triage_arrays(
    test_predictions["label_index"].to_numpy(),
    test_predictions["decision"].to_numpy(),
)
test_robust_metrics = threshold_candidate_metrics(
    test_predictions["label_index"].to_numpy(),
    test_predictions["p_occupied"].to_numpy(),
    T_EMPTY,
    T_OCCUPIED,
    BACKEND_EPSILON,
)
test_by_slot = grouped_triage_summary(test_predictions, "slot")
test_by_light = grouped_triage_summary(test_predictions, "captured_light")
test_by_frame, test_frame_metrics = frame_triage_summary(test_predictions)

test_slot_gate = bool((
    (test_by_slot["occupied_to_empty"] == 0)
    & (test_by_slot["empty_to_occupied"] == 0)
    & (test_by_slot["coverage"] >= 0.70)
).all())
test_gate_passed = bool(
    test_triage_metrics["occupied_to_empty"] == 0
    and test_triage_metrics["empty_to_occupied"] == 0
    and test_triage_metrics["known_accuracy"] == 1.0
    and test_triage_metrics["coverage"] >= 0.80
    and test_robust_metrics["robust_occupied_to_empty"] == 0
    and test_robust_metrics["robust_empty_to_occupied"] == 0
    and test_frame_metrics["unsafe_frames"] == 0
    and test_frame_metrics["cross_error_frames"] == 0
    and test_slot_gate
)
test_status = (
    "ONNX_BASELINE_READY"
    if test_gate_passed
    else "TEST_BASELINE_FAILED_DO_NOT_DEPLOY"
)

test_predictions.to_csv(
    LOCAL_RUN_DIR / "test_predictions.csv",
    index=False,
)
test_by_slot.to_csv(
    LOCAL_RUN_DIR / "test_by_slot.csv",
    index=False,
)
test_by_light.to_csv(
    LOCAL_RUN_DIR / "test_by_light.csv",
    index=False,
)
test_by_frame.to_csv(
    LOCAL_RUN_DIR / "test_by_frame.csv",
    index=False,
)
test_report_payload = {
    "schema_version": 1,
    "status": test_status,
    "run_id": RUN_ID,
    "thresholds_sha256": thresholds_sha256,
    "thresholds": {
        "t_empty": T_EMPTY,
        "t_occupied": T_OCCUPIED,
    },
    "binary_at_0p5": test_binary_metrics,
    "triage": test_triage_metrics,
    "robust_margin_metrics": test_robust_metrics,
    "by_slot": test_by_slot.to_dict(orient="records"),
    "by_light": test_by_light.to_dict(orient="records"),
    "by_frame": test_frame_metrics,
    "statistical_limitation": CLUSTERED_SAMPLE_LIMITATION,
    "test_consumption": test_lock_evidence,
    "retuning_from_test_forbidden": True,
    "limitation": (
        "s01/L00 same-site development baseline; "
        "not a final field-performance claim"
    ),
}
write_json(LOCAL_RUN_DIR / "test_report.json", test_report_payload)
write_json(
    LOCAL_RUN_DIR / "status.json",
    {
        "schema_version": 1,
        "status": test_status,
        "validation_gate_passed": True,
        "test_baseline_gate_passed": test_gate_passed,
        "field_test_passed": False,
        "tensorrt_verified": False,
    },
)

test_result_artifact_names = (
    "test_predictions.csv",
    "test_by_slot.csv",
    "test_by_light.csv",
    "test_by_frame.csv",
    "test_report.json",
    "status.json",
)
for file_name in test_result_artifact_names:
    copy_without_overwrite(
        LOCAL_RUN_DIR / file_name,
        DRIVE_REPORT_DIR / file_name,
    )

test_result_artifact_hashes = {}
for file_name in test_result_artifact_names:
    local_artifact = LOCAL_RUN_DIR / file_name
    drive_artifact = DRIVE_REPORT_DIR / file_name
    local_hash = sha256_file(local_artifact)
    if sha256_file(drive_artifact) != local_hash:
        raise RuntimeError(f"Drive test artifact SHA mismatch: {file_name}")
    test_result_artifact_hashes[file_name] = local_hash

test_completion_payload = {
    "schema_version": 1,
    "run_id": RUN_ID,
    "status": test_status,
    "test_gate_passed": test_gate_passed,
    "test_set_fingerprint": TEST_SET_FINGERPRINT,
    "thresholds_sha256": thresholds_sha256,
    "checkpoint_sha256": checkpoint_sha256,
    "onnx_sha256": onnx_sha256,
    "test_lock_evidence": test_lock_evidence,
    "test_artifacts": test_result_artifact_hashes,
    "completed_at_utc": datetime.now(timezone.utc).isoformat(),
}
LOCAL_TEST_COMPLETION_PATH = (
    LOCAL_RUN_DIR / "test_evaluation_complete.json"
)
DRIVE_TEST_COMPLETION_PATH = (
    DRIVE_REPORT_DIR / "test_evaluation_complete.json"
)
write_json(LOCAL_TEST_COMPLETION_PATH, test_completion_payload)
copy_without_overwrite(
    LOCAL_TEST_COMPLETION_PATH,
    DRIVE_TEST_COMPLETION_PATH,
)
if sha256_file(LOCAL_TEST_COMPLETION_PATH) != sha256_file(
    DRIVE_TEST_COMPLETION_PATH
):
    raise RuntimeError("Drive test completion marker SHA mismatch")

print("TEST BASELINE — thresholds were not changed")
print(json.dumps(test_triage_metrics, indent=2))
print("Test frames:", json.dumps(test_frame_metrics, indent=2))
display(test_by_slot)
print("Status:", test_status)
print(
    "주의: 이 test는 s01/L00 개발 기준선이며 현장 성능을 증명하지 않습니다."
)
print("PASS: test artifacts sealed with completion marker")

# %%
if not LOCAL_TEST_COMPLETION_PATH.is_file():
    raise RuntimeError("STOP: local test completion marker is missing")
if not DRIVE_TEST_COMPLETION_PATH.is_file():
    raise RuntimeError("STOP: Drive test completion marker is missing")
if sha256_file(LOCAL_TEST_COMPLETION_PATH) != sha256_file(
    DRIVE_TEST_COMPLETION_PATH
):
    raise RuntimeError("STOP: test completion markers do not match")

verified_test_completion = json.loads(
    LOCAL_TEST_COMPLETION_PATH.read_text(encoding="utf-8")
)
if verified_test_completion.get("run_id") != RUN_ID:
    raise RuntimeError("STOP: test completion marker belongs to another run")
if (
    verified_test_completion.get("test_set_fingerprint")
    != TEST_SET_FINGERPRINT
):
    raise RuntimeError("STOP: test fingerprint changed after evaluation")
if verified_test_completion.get("thresholds_sha256") != thresholds_sha256:
    raise RuntimeError("STOP: completion marker threshold hash mismatch")
LOCAL_THRESHOLDS_PATH = LOCAL_RUN_DIR / "thresholds.json"
DRIVE_THRESHOLDS_PATH = DRIVE_REPORT_DIR / "thresholds.json"
if sha256_file(LOCAL_THRESHOLDS_PATH) != thresholds_sha256:
    raise RuntimeError("STOP: frozen thresholds changed after test")
if sha256_file(DRIVE_THRESHOLDS_PATH) != thresholds_sha256:
    raise RuntimeError("STOP: Drive frozen thresholds changed after test")
frozen_thresholds = json.loads(
    LOCAL_THRESHOLDS_PATH.read_text(encoding="utf-8")
)
frozen_model = frozen_thresholds.get("model", {})
if frozen_model.get("checkpoint_sha256") != checkpoint_sha256:
    raise RuntimeError("STOP: threshold/checkpoint link is inconsistent")
if frozen_model.get("onnx_sha256") != onnx_sha256:
    raise RuntimeError("STOP: threshold/ONNX link is inconsistent")
if verified_test_completion.get("onnx_sha256") != sha256_file(
    LOCAL_ONNX_PATH
):
    raise RuntimeError("STOP: ONNX changed after test")
if verified_test_completion.get("checkpoint_sha256") != sha256_file(
    LOCAL_BEST_PATH
):
    raise RuntimeError("STOP: checkpoint changed after test")
if sha256_file(DRIVE_CHECKPOINT_DIR / "best.pt") != checkpoint_sha256:
    raise RuntimeError("STOP: Drive best checkpoint changed after test")

frozen_reproducibility_hashes = frozen_thresholds.get(
    "reproducibility_artifacts",
    {},
)
reproducibility_files = {
    "preprocessing_sha256": "preprocessing.json",
    "training_config_sha256": "training_config.json",
    "environment_sha256": "environment.json",
}
if set(frozen_reproducibility_hashes) != set(reproducibility_files):
    raise RuntimeError("STOP: reproducibility hash set is incomplete")
for hash_key, file_name in reproducibility_files.items():
    expected_hash = frozen_reproducibility_hashes[hash_key]
    if sha256_file(LOCAL_RUN_DIR / file_name) != expected_hash:
        raise RuntimeError(
            f"STOP: local reproducibility artifact changed: {file_name}"
        )
    if sha256_file(DRIVE_REPORT_DIR / file_name) != expected_hash:
        raise RuntimeError(
            f"STOP: Drive reproducibility artifact changed: {file_name}"
        )

frozen_code_snapshot = frozen_thresholds.get("code", {})
code_snapshot_files = (
    (
        "canonical_source_file",
        "canonical_source_sha256",
    ),
    (
        "notebook_snapshot_file",
        "notebook_snapshot_sha256",
    ),
    (
        "release_manifest_file",
        "release_manifest_sha256",
    ),
)
for file_key, hash_key in code_snapshot_files:
    file_name = frozen_code_snapshot.get(file_key)
    expected_hash = frozen_code_snapshot.get(hash_key)
    if not file_name or not expected_hash:
        raise RuntimeError("STOP: code snapshot evidence is incomplete")
    if sha256_file(LOCAL_RUN_DIR / file_name) != expected_hash:
        raise RuntimeError(f"STOP: local code snapshot changed: {file_name}")
    if sha256_file(DRIVE_REPORT_DIR / file_name) != expected_hash:
        raise RuntimeError(f"STOP: Drive code snapshot changed: {file_name}")
if notebook_semantic_sha256(
    LOCAL_RUN_DIR / frozen_code_snapshot["notebook_snapshot_file"]
) != frozen_code_snapshot.get("notebook_semantic_sha256"):
    raise RuntimeError("STOP: notebook code cells changed after training")

required_test_artifact_names = {
    "test_predictions.csv",
    "test_by_slot.csv",
    "test_by_light.csv",
    "test_by_frame.csv",
    "test_report.json",
    "status.json",
}
verified_test_artifacts = verified_test_completion.get("test_artifacts", {})
if set(verified_test_artifacts) != required_test_artifact_names:
    raise RuntimeError("STOP: completion marker artifact set is incomplete")
for file_name, expected_hash in verified_test_artifacts.items():
    if sha256_file(LOCAL_RUN_DIR / file_name) != expected_hash:
        raise RuntimeError(f"STOP: local test artifact changed: {file_name}")
    if sha256_file(DRIVE_REPORT_DIR / file_name) != expected_hash:
        raise RuntimeError(f"STOP: Drive test artifact changed: {file_name}")

verified_test_gate_passed = (
    verified_test_completion.get("test_gate_passed") is True
)
verified_test_status = verified_test_completion.get("status")
expected_verified_status = (
    "ONNX_BASELINE_READY"
    if verified_test_gate_passed
    else "TEST_BASELINE_FAILED_DO_NOT_DEPLOY"
)
if verified_test_status != expected_verified_status:
    raise RuntimeError("STOP: completion marker status is inconsistent")

verified_test_report = json.loads(
    (LOCAL_RUN_DIR / "test_report.json").read_text(encoding="utf-8")
)
verified_status_report = json.loads(
    (LOCAL_RUN_DIR / "status.json").read_text(encoding="utf-8")
)
if verified_test_report.get("status") != verified_test_status:
    raise RuntimeError("STOP: test report status is inconsistent")
if (
    verified_status_report.get("test_baseline_gate_passed")
    is not verified_test_gate_passed
):
    raise RuntimeError("STOP: status report gate result is inconsistent")

verified_lock_evidence = verified_test_completion.get(
    "test_lock_evidence",
    {},
)
if sha256_file(TEST_LOCK_PATH) != verified_lock_evidence.get(
    "set_lock",
    {},
).get("sha256"):
    raise RuntimeError("STOP: test-set lock changed after evaluation")
verified_source_locks = verified_lock_evidence.get(
    "source_frame_locks",
    [],
)
if len(verified_source_locks) != len(SOURCE_FRAME_LOCK_PATHS):
    raise RuntimeError("STOP: source-frame lock evidence is incomplete")
for lock_path, lock_record in zip(
    SOURCE_FRAME_LOCK_PATHS,
    verified_source_locks,
):
    expected_relative_path = lock_path.relative_to(DRIVE_ROOT).as_posix()
    if lock_record.get("relative_path") != expected_relative_path:
        raise RuntimeError("STOP: source-frame lock path changed")
    if sha256_file(lock_path) != lock_record.get("sha256"):
        raise RuntimeError(f"STOP: source-frame lock changed: {lock_path}")

if verified_test_gate_passed:
    frozen_preprocessing = json.loads(
        (LOCAL_RUN_DIR / "preprocessing.json").read_text(encoding="utf-8")
    )
    test_report_sha256 = verified_test_completion["test_artifacts"][
        "test_report.json"
    ]
    deployment_metadata_payload = {
        "schema_version": 1,
        "status": "ONNX_BASELINE_READY",
        "approval_scope": "same-site development ONNX baseline only",
        "deployment_authorized": False,
        "field_test_passed": False,
        "tensorrt_verified": False,
        "temporal_filter_validated": False,
        "camera_fault_blocking_validated": False,
        "run_id": verified_test_completion["run_id"],
        "model": {
            "architecture": frozen_model["architecture"],
            "pooling": frozen_model["pooling"],
            "checkpoint_sha256": frozen_model["checkpoint_sha256"],
            "onnx_file": frozen_model["onnx_file"],
            "onnx_sha256": frozen_model["onnx_sha256"],
        },
        "preprocessing": frozen_preprocessing,
        "thresholds": frozen_thresholds["thresholds"],
        "thresholds_sha256": verified_test_completion["thresholds_sha256"],
        "backend_epsilon": frozen_thresholds["selection"]["epsilon_total"],
        "test_report_file": "test_report.json",
        "test_report_sha256": test_report_sha256,
        "test_status": verified_test_status,
        "test_consumption": verified_test_completion[
            "test_lock_evidence"
        ],
        "code_snapshot": frozen_code_snapshot,
        "reproducibility_artifacts": frozen_reproducibility_hashes,
        "data": {
            **frozen_thresholds["data"],
            "test_set_fingerprint": verified_test_completion[
                "test_set_fingerprint"
            ],
        },
        "roi_calibration": roi_config,
        "statistical_limitation": frozen_thresholds[
            "statistical_limitation"
        ],
        "deployment_limitations": [
            "field test has not passed",
            "TensorRT parity has not been verified",
            "temporal filter and camera-fault command blocking are not validated",
        ],
    }
    write_json_if_identical(
        LOCAL_RUN_DIR / "deployment_metadata.json",
        deployment_metadata_payload,
    )
    copy_without_overwrite(
        LOCAL_RUN_DIR / "deployment_metadata.json",
        DRIVE_REPORT_DIR / "deployment_metadata.json",
    )
    publication_bundle_sources = (
        LOCAL_ONNX_PATH,
        LOCAL_RUN_DIR / "preprocessing.json",
        LOCAL_RUN_DIR / "thresholds.json",
        LOCAL_RUN_DIR / "test_report.json",
        LOCAL_RUN_DIR / "deployment_metadata.json",
    )
    for bundle_source in publication_bundle_sources:
        copy_without_overwrite(
            bundle_source,
            DRIVE_ONNX_DIR / bundle_source.name,
        )
        if sha256_file(bundle_source) != sha256_file(
            DRIVE_ONNX_DIR / bundle_source.name
        ):
            raise RuntimeError(
                f"Published ONNX bundle SHA mismatch: {bundle_source.name}"
            )

artifact_records = []
FINAL_INDEX_FILES = {"artifact_manifest.json", "SHA256SUMS.txt"}
for category, directory in (
    ("checkpoint", DRIVE_CHECKPOINT_DIR),
    ("onnx", DRIVE_ONNX_DIR),
    ("report", DRIVE_REPORT_DIR),
):
    for artifact_path in sorted(directory.iterdir()):
        if (
            artifact_path.is_file()
            and artifact_path.name not in FINAL_INDEX_FILES
        ):
            artifact_records.append({
                "category": category,
                "relative_path": (
                    artifact_path.relative_to(DRIVE_ROOT).as_posix()
                ),
                "bytes": artifact_path.stat().st_size,
                "sha256": sha256_file(artifact_path),
            })

artifact_manifest_path = DRIVE_REPORT_DIR / "artifact_manifest.json"
write_json_if_identical(
    artifact_manifest_path,
    {
        "schema_version": 1,
        "run_id": verified_test_completion["run_id"],
        "status": verified_test_status,
        "artifacts": artifact_records,
    },
)

checksum_lines = [
    f"{record['sha256']}  {record['relative_path']}"
    for record in artifact_records
]
checksum_path = DRIVE_REPORT_DIR / "SHA256SUMS.txt"
write_text_if_identical(
    checksum_path,
    "\n".join(checksum_lines) + "\n",
)

print("Checkpoint directory:", DRIVE_CHECKPOINT_DIR)
print("ONNX directory:", DRIVE_ONNX_DIR)
print("Report directory:", DRIVE_REPORT_DIR)
print("Artifact count:", len(artifact_records))

if not verified_test_gate_passed:
    raise RuntimeError(
        "STOP: test baseline gate failed. "
        "임계값을 test에 맞춰 수정하지 말고 새 학습/검증 사이클로 돌아가세요."
    )

print("PASS: ONNX_BASELINE_READY")
print("다음 단계: Jetson에서 TensorRT FP16 엔진 생성과 ONNX 대비 검증")

# %% [markdown]
# ## 완료 판정
#
# 마지막 셀에서 `PASS: ONNX_BASELINE_READY`가 출력되어야 한다.
#
# 이것은 **개발 기준 ONNX 준비 완료**를 의미한다. 다음 항목이 끝나기 전에는 배포 완료가 아니다.
#
# - 별도 날짜·현장 조명 test
# - Jetson TensorRT FP16 엔진 생성
# - ONNX/TensorRT 수치 및 상태 판정 비교
# - 최근 5프레임 중 4프레임 시간 필터 검증
# - 카메라 장애 시 `UNKNOWN/CAMERA_FAULT` 및 STM32 명령 차단 검증
