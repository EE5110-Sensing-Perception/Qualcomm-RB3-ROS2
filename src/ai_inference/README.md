# AI Inference Package
# UNDER CONSTRUCTION...
This package provides AI inference capabilities for the QIRP device using TensorFlow Lite models optimized for Qualcomm Snapdragon processors.

## Overview

The AI Inference package includes:
- **YOLO Object Detection**: Real-time object detection using YOLOv8 models
- **TensorFlow Lite Integration**: Optimized for ARM64 architecture
- **ROS2 Integration**: Standard ROS2 message types and topics

## Features

- ✅ **YOLOv8 Object Detection** with 80 COCO classes
- ✅ **TensorFlow Lite** optimized for QIRP hardware
- ✅ **Configurable parameters** via YAML files
- ✅ **ROS2 standard messages** (sensor_msgs, vision_msgs)
- ✅ **Launch file support** for easy deployment

## Package Structure

```
ai_inference/
├── ai_inference/              # Python package
│   ├── __init__.py
│   └── yolo_detector.py       # Main detection node
├── config/                    # Configuration files
│   └── yolo_config.yaml       # YOLO parameters
├── launch/                    # Launch files
│   └── yolo_detector_launch.py
├── models/                    # AI models
│   └── yolov8_det_float.tflite
├── resource/                  # Package resources
├── package.xml               # Package manifest
├── setup.py                  # Python setup
└── README.md                 # This file
```

## Installation

### Prerequisites

```bash
# Install TensorFlow Lite (if not already installed)
pip3 install tflite-runtime --break-system-packages

# Or install full TensorFlow
pip3 install tensorflow --break-system-packages
```

### Build Package

```bash
# Build the package
colcon build --packages-select ai_inference

# Source the workspace
source install/setup.bash
```

## Usage

### Run YOLO Detector

```bash
# Run with default configuration
ros2 run ai_inference yolo_detector

# Run with launch file
ros2 launch ai_inference yolo_detector_launch.py

# Run with custom config
ros2 launch ai_inference yolo_detector_launch.py config_file:=/path/to/config.yaml
```

### Topics

**Subscribed:**
- `/camera/image_raw` (sensor_msgs/Image) - Input camera images

**Published:**
- `/ai_inference/detections` (vision_msgs/Detection2DArray) - Detection results

### Parameters

Configure via `config/yolo_config.yaml`:

```yaml
yolo_detector:
  ros__parameters:
    model_path: "yolov8_det_float.tflite"
    confidence_threshold: 0.5
    input_width: 640
    input_height: 640
    max_detections: 100
    use_gpu: true
```

## Configuration

### Model Configuration

- **model_path**: Path to TensorFlow Lite model file
- **confidence_threshold**: Minimum confidence for detections (0.0-1.0)
- **nms_threshold**: Non-Maximum Suppression threshold
- **input_width/height**: Input image dimensions

### Performance Settings

- **max_detections**: Maximum number of detections per frame
- **use_gpu**: Enable GPU acceleration (if available)

### Class Names

The package includes all 80 COCO class names for object detection:
- person, bicycle, car, motorcycle, airplane, bus, train, truck, boat, etc.

## Adding New Models

1. Place model files in `models/` directory
2. Update `config/yolo_config.yaml` with new model path
3. Modify `yolo_detector.py` if needed for different model types

## Troubleshooting

### Common Issues

**TensorFlow Lite not found:**
```bash
pip3 install tflite-runtime --break-system-packages
```

**Model file not found:**
- Check model path in configuration
- Ensure model file exists in `models/` directory

**No detections:**
- Check confidence threshold
- Verify input image topic is publishing
- Check model compatibility

### Debug Commands

```bash
# Check if node is running
ros2 node list | grep yolo

# Monitor detection topics
ros2 topic echo /ai_inference/detections

# Check node parameters
ros2 param list /yolo_detector
ros2 param get /yolo_detector confidence_threshold
```

## Performance Optimization

### For QIRP Device

1. **Use quantized models** (float16 or int8) for better performance
2. **Enable GPU acceleration** if available
3. **Adjust input resolution** based on requirements
4. **Optimize confidence threshold** for your use case

### Model Selection

- **yolov8_det_float.tflite**: Full precision, higher accuracy
- **yolov8_det_int8.tflite**: Quantized, faster inference
- **yolov8n_det.tflite**: Nano version, fastest inference

## Future Enhancements

- [ ] Support for additional model types (ONNX, PyTorch)
- [ ] Multi-model inference pipeline
- [ ] Custom class training support
- [ ] Performance benchmarking tools
- [ ] Real-time visualization

## License

This package is part of the QIRP workspace and follows the same licensing terms.
