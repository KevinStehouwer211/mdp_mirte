from pathlib import Path
from ultralytics import YOLO

SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_DIR = SCRIPT_DIR.parent

DATA_YAML = PACKAGE_DIR / "datasets" / "awesome-flowers" / "data.yaml"
MODEL_PATH = PACKAGE_DIR / "models" / "best.pt"
IMG_SIZE = 640


def main():
    model = YOLO(str(MODEL_PATH))

    model.val(
        data=str(DATA_YAML),
        conf=0.43,
        imgsz=IMG_SIZE,
        project=str(PACKAGE_DIR / "runs" / "flower_validation"),
        name="yolov8n_flowers_val",
        plots=True,
    )


if __name__ == "__main__":
    main()