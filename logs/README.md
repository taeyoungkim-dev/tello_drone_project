# Flight Logs

This folder contains performance logs for comparing optimization results.

## Log Files

### Naming Convention
- `flight_log_opencv_YYYYMMDD_HHMMSS.csv` - Original OpenCV implementation (baseline)
- `flight_log_opencv_optimized_YYYYMMDD_HHMMSS.csv` - Optimized OpenCV with Producer-Consumer pattern
- `flight_log_yolov8_YYYYMMDD_HHMMSS.csv` - Original YOLOv8 implementation (baseline)
- `flight_log_yolov8_optimized_YYYYMMDD_HHMMSS.csv` - Optimized YOLOv8 with Producer-Consumer pattern

## Data Format

Each CSV file contains the following columns:

| Column | Description |
|--------|-------------|
| `FrameCount` | Sequential frame number |
| `Timestamp` | Unix timestamp (seconds) |
| `Latency_ms` | Frame age in milliseconds (0 for non-optimized versions) |
| `FPS` | Processing frames per second |

## Usage

To analyze and visualize the performance improvements:

```bash
python analyze_performance.py
```

This will generate a `performance_comparison.png` file showing:
- FPS comparison over time
- Average FPS improvement
- Frame latency (optimized versions only)
