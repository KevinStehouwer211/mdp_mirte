from pathlib import Path
from ultralytics import YOLO

SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_DIR = SCRIPT_DIR.parent

DATA_YAML = PACKAGE_DIR / "datasets" / "awesome-flowers" / "data.yaml"
MODEL = "yolov8n.pt"
EPOCHS = 50
IMG_SIZE = 640


def main():
    model = YOLO(MODEL)

    model.train(
        data=str(DATA_YAML),
        epochs=EPOCHS,
        imgsz=IMG_SIZE,
        project=str(PACKAGE_DIR / "runs" / "flower_detection"),
        name="yolov8n_flowers",
    )


if __name__ == "__main__":
    main()