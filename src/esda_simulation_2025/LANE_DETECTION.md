# Lane Detection Feature

## Overview
The lane detection system uses computer vision to identify white lane markings from the robot's camera feed and publishes them as visual markers on the SLAM map.

## How to Use

### 1. Launch via UI (Recommended)
1. Run the UI launcher: `python3 src/esda_simulation_2025/scripts/ui_launch.py`
2. Build the workspace (button 1)
3. Launch the simulation (button 2)
4. **Check the "Enable Lane Detection (with SLAM)" checkbox**
5. Click "Launch SLAM"

The lane detection will automatically start, showing:
- A visualization window with 4 panels:
  - **Top-left**: Original camera feed
  - **Top-right**: White pixel detection mask
  - **Bottom-left**: Edge detection (Region of Interest)
  - **Bottom-right**: Detected lane lines (green) overlaid on the image
- Lane markers published to `/lane_markers` topic (visible in RViz)

### 2. Manual Launch
If you prefer to launch manually:

```bash
# Source the workspace
source install/setup.bash

# Launch lane detection node
ros2 run esda_simulation_2025 lane_detection.py
```

## Visualizing in RViz2

To see the detected lanes in RViz:

1. Launch RViz2
2. Add a **MarkerArray** display:
   - Click "Add" button
   - Select "By topic"
   - Choose `/lane_markers`
3. The detected lanes will appear as yellow line segments in the camera frame

## Parameters

You can customize the lane detection behavior by passing parameters:

```bash
ros2 run esda_simulation_2025 lane_detection.py --ros-args \
  -p show_visualization:=true \
  -p white_threshold_low:=200 \
  -p white_threshold_high:=255 \
  -p min_line_length:=50 \
  -p max_line_gap:=50
```

### Parameter Descriptions:
- `show_visualization` (bool, default: true): Show the OpenCV visualization window
- `white_threshold_low` (int, default: 200): Lower threshold for white color detection (0-255)
- `white_threshold_high` (int, default: 255): Upper threshold for white color detection (0-255)
- `min_line_length` (int, default: 50): Minimum line length in pixels for Hough transform
- `max_line_gap` (int, default: 50): Maximum gap between line segments to connect them

## Algorithm Details

The lane detection uses a lightweight computer vision pipeline:

1. **Color Filtering**: Converts image to grayscale and applies threshold to isolate white pixels
2. **Edge Detection**: Uses Canny edge detector to find edges
3. **Region of Interest**: Focuses on lower 40% of image where ground appears
4. **Line Detection**: Uses Hough Line Transform to detect straight lines
5. **3D Projection**: Projects detected image lines to 3D world coordinates
6. **Publishing**: Publishes as MarkerArray for visualization in SLAM map

## Topics

### Subscribed Topics:
- `/camera_raw` (sensor_msgs/Image): Raw camera feed from simulation

### Published Topics:
- `/lane_markers` (visualization_msgs/MarkerArray): Detected lane lines as 3D markers

## Troubleshooting

### No lanes detected
- Check camera feed: `ros2 topic echo /camera_raw --once`
- Adjust white threshold parameters if ground texture is different
- Ensure there are white markings visible in the camera view

### Visualization window doesn't appear
- Set parameter: `show_visualization:=true`
- Ensure X11 forwarding is working (if using remote connection)

### Markers not visible in RViz
- Add MarkerArray display with topic `/lane_markers`
- Check frame: Markers are published in `camera_link` frame
- Ensure TF tree is complete

## Performance

The lane detection node is optimized for real-time performance:
- Processing rate: ~10-30 Hz (depending on image size and line complexity)
- CPU usage: Low (simple OpenCV operations)
- Memory: Minimal (<100MB)

## Future Enhancements

Possible improvements:
- Lane following controller (use detected lanes for autonomous navigation)
- Lane departure warning
- Curved lane detection (currently only straight lines)
- Machine learning-based detection for better accuracy
- Integration with Nav2 costmap as obstacles
