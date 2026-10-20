"""Start the YOLO lane+cone segmentation node against the simulated camera.

    ros2 launch esda_simulation_2025 lane_seg.launch.py
    ros2 launch esda_simulation_2025 lane_seg.launch.py detector:=classic
    ros2 launch esda_simulation_2025 lane_seg.launch.py camera_mount:=front

This is the only detector that produces CONES. The classic thresholds are
lane-only by construction, which is why this node exists: a white-striped
barrel read as lane paint walls off the drivable gap.

WHAT IT PUBLISHES
    /lane/points        linked, gap-free lane boundary
    /lane/points_raw    the same before linking
    /cone/points        cones, lidar-confirmed, as filled discs
    /lane/lines         MarkerArray, one strip per lane line
    /lane/debug_image   annotated overlay

NAV2 DOES NOT CONSUME THESE YET. The stack drives on /scan_fused, which
lane_detection.py and lane_detection_esda.py publish and this node does not.
Run this to SEE what the network detects and to score it; adding
/lane/points and /cone/points as costmap observation sources is a separate,
deliberate change -- research/vision/nav2_vision_costmap_snippet.yaml has
the block, and it needs a scored run before anyone relies on it.

CAMERA GEOMETRY
Projection needs the camera's real pose, and getting it wrong fails
SILENTLY: points still come out, just in the wrong place on the ground. So
the numbers below are derived from the xacro chain rather than measured off
a screenshot, and `camera_mount` here must match the one the sim was
launched with. launch_sim.launch.py defaults to pole_top.

    robot_core_ref.xacro  chassis_joint: base_link -> chassis  xyz -0.965423 0 0
    camera.xacro          pole_top: chassis -> camera_link  xyz 0.560407 0.10 1.30  rpy 0 0.2618 0
                          front:    chassis -> camera_link  xyz 0.8      0    0.315 rpy 0 0      0
    base_link rides at the wheel axle, wheel_radius 0.1625 above the ground.

so, above GROUND and relative to base_link:

    pole_top   height 1.30  + 0.1625 = 1.4625   x -0.405016   pitch 15 deg down
    front      height 0.315 + 0.1625 = 0.4775   x -0.165423   pitch  0

Sign warning: cam_pitch_deg here is POSITIVE for a downward tilt, matching
the xacro. lane_detection_esda.py's camera_mount_pitch is NEGATIVE for the
same tilt -- it works in the optical frame, where Y points down. The two
numbers describe the same camera.
"""
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = "esda_simulation_2025"

# chassis -> camera_link, and the chassis offset that turns it into base_link
CHASSIS_X = -0.965423
WHEEL_RADIUS = 0.1625

MOUNTS = {
    # name:      (camera z on chassis, camera x on chassis, pitch deg down)
    "pole_top": (1.30, 0.560407, 15.0),
    "front": (0.315, 0.8, 0.0),
}

# camera.xacro: 640x480, horizontal_fov 1.089 rad, square pixels.
IMG_W, IMG_H, HFOV = 640.0, 480.0, 1.089
FX = (IMG_W / 2.0) / math.tan(HFOV / 2.0)


def _launch_setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    mount = arg("camera_mount")
    if mount not in MOUNTS:
        raise RuntimeError(
            "camera_mount must be one of %s, got %r"
            % (sorted(MOUNTS), mount))
    cam_z, cam_x, pitch_deg = MOUNTS[mount]

    share = get_package_share_directory(PKG)
    config = arg("config") or os.path.join(share, "vision", "vision_config.yaml")

    argv = [
        "--config", config,
        "--detector", arg("detector"),
        "--mode", arg("mode"),
        "--image-topic", arg("image_topic"),
        "--scan-topic", arg("scan_topic"),
        "--cam-height", "%.6f" % (cam_z + WHEEL_RADIUS),
        "--cam-x-offset", "%.6f" % (cam_x + CHASSIS_X),
        "--cam-pitch-deg", "%.4f" % pitch_deg,
        "--fx", "%.4f" % FX,
        "--cx", "%.1f" % (IMG_W / 2.0),
        "--cy", "%.1f" % (IMG_H / 2.0),
    ]
    if arg("device"):
        argv += ["--device", arg("device")]
    if arg("weights"):
        argv += ["--weights", arg("weights")]
    if arg("classic_method"):
        argv += ["--classic-method", arg("classic_method")]
    if arg("link").lower() in ("false", "0", "no"):
        argv += ["--no-link"]

    return [Node(
        package=PKG,
        executable="lane_seg_node.py",
        name="lane_seg_node",
        output="screen",
        arguments=argv,
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "camera_mount", default_value="pole_top",
            choices=sorted(MOUNTS),
            description="MUST match the mount the sim was launched with; "
                        "launch_sim.launch.py defaults to pole_top"),
        DeclareLaunchArgument(
            "detector", default_value="yolo", choices=["yolo", "classic"],
            description="yolo is the only one that produces cones"),
        DeclareLaunchArgument(
            "mode", default_value="full",
            choices=["full", "lane_cone", "lane_only"],
            description="which classes get published; no retrain needed"),
        DeclareLaunchArgument(
            "image_topic", default_value="/camera/left/image_raw"),
        DeclareLaunchArgument(
            "scan_topic", default_value="/scan",
            description="raw lidar, used to confirm cones. NOT /scan_fused: "
                        "that already has lane obstacles baked in by the "
                        "classic detector, and gating cones against them "
                        "would confirm vision with vision."),
        DeclareLaunchArgument(
            "device", default_value="",
            description="'0' for the first GPU, 'cpu' to force CPU; empty "
                        "uses the config, which falls back to cpu anyway "
                        "when no CUDA is visible"),
        DeclareLaunchArgument(
            "weights", default_value="",
            description="empty uses the weight shipped in the package share"),
        DeclareLaunchArgument(
            "classic_method", default_value="",
            description="otsu | value | adaptive | tophat, when detector:=classic"),
        DeclareLaunchArgument(
            "config", default_value="",
            description="empty uses the package's installed vision_config.yaml"),
        DeclareLaunchArgument("link", default_value="true"),
        OpaqueFunction(function=_launch_setup),
    ])
