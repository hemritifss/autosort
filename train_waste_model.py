# ═══════════════════════════════════════════════
# TRAINING PIPELINE
# ═══════════════════════════════════════════════

from ultralytics import YOLO
import os

# ── DATASET STRUCTURE ─────────────────────────
"""
dataset/
├── images/
│   ├── train/    (80%)
│   ├── val/      (15%)
│   └── test/     (5%)
└── labels/       (YOLO format .txt)
    ├── train/
    ├── val/
    └── test/

Sources for data:
1. TACO dataset (open source waste images)
2. OpenLitterMap
3. Your own photos! (most important)
4. Roboflow waste datasets
"""

# ── DATASET CONFIG ────────────────────────────
dataset_yaml = """
path: ./dataset
train: images/train
val: images/val
test: images/test

nc: 16  # number of classes
names:
  0: PET_clear
  1: PET_colored
  2: HDPE
  3: PP
  4: PS
  5: paper
  6: cardboard
  7: aluminum_can
  8: steel_can
  9: glass_bottle
  10: film_plastic
  11: tetra_pak
  12: battery
  13: hazardous
  14: organic
  15: unknown
"""

with open('waste_dataset.yaml', 'w') as f:
    f.write(dataset_yaml)

# ── TRAINING ──────────────────────────────────
def train_model():
    
    # Start with pretrained YOLOv8
    model = YOLO('yolov8n.pt')  # nano = fastest for Jetson
    
    results = model.train(
        data='waste_dataset.yaml',
        epochs=100,
        imgsz=640,
        batch=16,
        patience=20,           # early stopping
        device=0,              # GPU
        
        # Augmentation (important for robustness!)
        hsv_h=0.015,           # color augment
        hsv_s=0.7,
        hsv_v=0.4,
        degrees=45,            # rotation
        translate=0.1,
        scale=0.5,             # zoom
        fliplr=0.5,            # horizontal flip
        mosaic=1.0,            # mosaic augment
        mixup=0.2,             # mixup augment
        
        # Optimization
        optimizer='AdamW',
        lr0=0.001,
        weight_decay=0.0005,
        
        # Save
        project='waste_sorter',
        name='v1',
        save_period=10
    )
    
    return results

# ── EXPORT FOR JETSON ─────────────────────────
def export_for_jetson():
    model = YOLO('waste_sorter/v1/weights/best.pt')
    
    # Export to TensorRT for maximum speed on Jetson
    model.export(
        format='engine',      # TensorRT
        device=0,
        half=True,            # FP16 (faster!)
        simplify=True
    )
    
    print("✅ Exported to TensorRT!")
    print("Expected speed: ~30-60 FPS on Jetson Nano")

# ── QUICK DATA COLLECTION TOOL ────────────────
def collect_data_tool():
    """
    Tool to quickly label images from your camera
    """
    import cv2
    
    cap = cv2.VideoCapture(0)
    count = 0
    
    classes = [
        'PET', 'HDPE', 'PP', 'paper', 
        'aluminum', 'steel', 'battery', 'other'
    ]
    
    print("Keys: p=PET, h=HDPE, r=PP, a=paper,")
    print("      l=aluminum, s=steel, b=battery, o=other")
    print("      SPACE=capture, q=quit")
    
    current_class = 'unknown'
    
    while True:
        ret, frame = cap.read()
        
        cv2.putText(frame, f"Class: {current_class}", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                   1, (0,255,0), 2)
        cv2.putText(frame, f"Count: {count}",
                   (10, 70), cv2.FONT_HERSHEY_SIMPLEX,
                   1, (0,255,0), 2)
        
        cv2.imshow('Data Collection', frame)
        
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('p'): current_class = 'PET'
        elif key == ord('h'): current_class = 'HDPE'
        elif key == ord('r'): current_class = 'PP'
        elif key == ord('a'): current_class = 'paper'
        elif key == ord('l'): current_class = 'aluminum'
        elif key == ord('s'): current_class = 'steel'
        elif key == ord('b'): current_class = 'battery'
        elif key == ord('o'): current_class = 'other'
        elif key == ord(' '):
            # Save image
            os.makedirs(
                f'dataset/images/train/{current_class}', 
                exist_ok=True
            )
            filename = (f'dataset/images/train/'
                       f'{current_class}/'
                       f'{current_class}_{count:04d}.jpg')
            cv2.imwrite(filename, frame)
            count += 1
            print(f"✅ Saved: {filename}")
        elif key == ord('q'):
            break
    
    cap.release()


if __name__ == '__main__':
    train_model()
    export_for_jetson()
