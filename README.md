
# ROS2 ZED2 YOLO Node

## Install Dependencies
```bash
pip install opencv-python numpy torch torchvision ultralytics pyzed
```

## Download Model
```bash
cd src/zed2_yolo_node/
mkdir models
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8s.pt -O models/yolov8s.pt
```

## Build & Run
```bash
# Build
colcon build --packages-select zed2_yolo_node
source install/setup.bash

# Run
ros2 run zed2_yolo_node main
```

## Troubleshooting
- Package not found? Run `colcon build` again and `source install/setup.bash`
- Import errors? Install missing packages with `pip install`
