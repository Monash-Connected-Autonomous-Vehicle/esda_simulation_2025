#!/usr/bin/env python3
"""
Camera calibration for ArUco marker pose estimation (e.g. following the ADR).

ArUco pose (distance/bearing to a marker) is only as good as the camera's
intrinsics, so this tool produces them and checks them. No ROS needed:
OpenCV >= 4.7 (new ArUco API), plus pyzed for the ZED options.

Subcommands
-----------
  board        Write a printable ChArUco calibration board (PNG).
  identify     Work out an existing ChArUco board's layout (dictionary, grid,
               marker/square ratio, legacy pattern) from the camera, and
               print the calibrate command for it.
  zed-factory  Save the ZED's factory calibration (left camera). ZEDs are
               calibrated at the factory; for the rectified images the SDK
               serves, these intrinsics are exact and distortion is zero.
               Usually the best choice for a ZED.
  calibrate    Calibrate from a ChArUco board seen live (ZED or any OpenCV
               camera) or from saved images. Frames are auto-captured when the
               board is well seen in a new position.
  verify       Live ArUco detection with the saved calibration: draws each
               marker's axes and prints its distance and bearing.

Output is the ROS camera_info YAML format (camera_calibration /
camera_info_manager), so the same file can feed a ROS node later.

Typical use
-----------
  # Webcam on /dev/video0 (Logitech Brio 100, the default source, 1280x720):
  python3 tools/aruco_camera_calibration.py board -o charuco_board.png      # print it
  python3 tools/aruco_camera_calibration.py calibrate --square-length 0.035 \\
      --board-marker-length 0.026 -o src/esda_simulation_2025/config/brio100_1280x720.yaml
  python3 tools/aruco_camera_calibration.py verify \\
      --calib src/esda_simulation_2025/config/brio100_1280x720.yaml

  # ZED: just export the factory calibration, then check it on a marker
  python3 tools/aruco_camera_calibration.py zed-factory --resolution HD720 -o config/zed_left_HD720.yaml
  python3 tools/aruco_camera_calibration.py verify --source zed --calib config/zed_left_HD720.yaml

  # Headless (over SSH): no window, frames saved for review
  python3 tools/aruco_camera_calibration.py calibrate --headless --save-frames /tmp/calib_frames -o cam.yaml
  # Re-run on saved frames
  python3 tools/aruco_camera_calibration.py calibrate --images '/tmp/calib_frames/*.png' -o cam.yaml

ADR markers (the verify defaults) come from aruco_generate.py:
DICT_4X4_1000, ids 420-424, a 190 mm purple marker with a 10 mm white
border on a page cut to 210 mm. Against a background darker than the paper
the detector can lock onto the page edge (distance ~10 % short); verify spots
that and steps in to the real marker edge (--page-border). The camera
calibration itself doesn't depend on the marker size - only the pose does.

Measure the PRINTED board/markers with a ruler and pass the real sizes
(--square-length / --marker-length, metres): printers scale pages, and pose
distances scale with them.
"""

import argparse
import glob
import math
import os
import sys
import time

import cv2
import numpy as np
import yaml

# ---------------------------------------------------------------------------
# Board / dictionary helpers
# ---------------------------------------------------------------------------

# The calibration board deliberately defaults to a different dictionary from
# the usual 4x4 marker sets, so a board in view is never mistaken for an ADR
# marker (and vice versa).
DEFAULT_BOARD_DICT = 'DICT_5X5_100'


def get_dictionary(name):
    if not hasattr(cv2.aruco, name):
        valid = [n for n in dir(cv2.aruco) if n.startswith('DICT_')]
        sys.exit(f'Unknown ArUco dictionary {name!r}. Options: {", ".join(valid)}')
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def make_board(args):
    board = cv2.aruco.CharucoBoard(
        (args.squares_x, args.squares_y), args.square_length, args.board_marker_length,
        get_dictionary(args.board_dict))
    # Boards from OpenCV < 4.6 and many commercial plates (e.g. calib.io) with
    # an even number of rows use the old layout - `identify` tells you.
    board.setLegacyPattern(args.legacy)
    return board


def add_board_args(p):
    g = p.add_argument_group('ChArUco board (must match the printed board)')
    g.add_argument('--squares-x', type=int, default=7, help='squares across (default 7)')
    g.add_argument('--squares-y', type=int, default=5, help='squares down (default 5)')
    g.add_argument('--square-length', type=float, default=0.035,
                   help='MEASURED square side on the print, metres (default 0.035)')
    g.add_argument('--board-marker-length', type=float, default=0.026,
                   help='MEASURED marker side inside a square, metres (default 0.026)')
    g.add_argument('--board-dict', default=DEFAULT_BOARD_DICT,
                   help=f'board dictionary (default {DEFAULT_BOARD_DICT})')
    g.add_argument('--legacy', action='store_true',
                   help='old (pre-OpenCV 4.6) ChArUco layout - use if identify says so')


# ---------------------------------------------------------------------------
# Camera sources
# ---------------------------------------------------------------------------

class ZedSource:
    """Left image of a ZED via the ZED SDK (pyzed)."""

    def __init__(self, resolution='HD720', fps=30, raw=False):
        import pyzed.sl as sl
        self.sl = sl
        self.cam = sl.Camera()
        init = sl.InitParameters()
        init.camera_resolution = getattr(sl.RESOLUTION, resolution)
        init.camera_fps = fps
        init.depth_mode = sl.DEPTH_MODE.NONE
        err = self.cam.open(init)
        if err != sl.ERROR_CODE.SUCCESS:
            sys.exit(f'Could not open the ZED: {err}')
        # Rectified left is what the SDK (and the ZED ROS wrapper) publish;
        # raw is only useful to calibrate the lens distortion yourself.
        self.view = sl.VIEW.LEFT_UNRECTIFIED if raw else sl.VIEW.LEFT
        self.mat = sl.Mat()
        self.runtime = sl.RuntimeParameters()
        self.ended = False

    def read(self):
        if self.cam.grab(self.runtime) != self.sl.ERROR_CODE.SUCCESS:
            return None
        self.cam.retrieve_image(self.mat, self.view)
        return cv2.cvtColor(self.mat.get_data(), cv2.COLOR_BGRA2BGR)

    def factory_calibration(self, raw=False):
        info = self.cam.get_camera_information().camera_configuration
        params = info.calibration_parameters_raw if raw else info.calibration_parameters
        left = params.left_cam
        k = np.array([[left.fx, 0, left.cx], [0, left.fy, left.cy], [0, 0, 1]], dtype=np.float64)
        d = np.array(left.disto[:5], dtype=np.float64)
        size = (info.resolution.width, info.resolution.height)
        return k, d, size

    def close(self):
        self.cam.close()


class OpenCvSource:
    """A V4L2 webcam (/dev/videoN or index N) or any URL/file OpenCV can open."""

    def __init__(self, spec, width=None, height=None, fourcc='MJPG'):
        device = spec[len('/dev/video'):] if spec.startswith('/dev/video') else spec
        if device.isdigit():
            # V4L2 + MJPG: webcams like the Brio 100 only reach 30 fps at
            # 720p/1080p compressed; uncompressed YUYV is ~5-10 fps there.
            self.cap = cv2.VideoCapture(int(device), cv2.CAP_V4L2)
            if fourcc:
                self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        else:
            self.cap = cv2.VideoCapture(spec)
        if not self.cap.isOpened():
            sys.exit(f'Could not open camera source {spec!r}')
        if width:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        if height:
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if device.isdigit():
            w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            print(f'Camera {spec}: {w}x{h} @ {self.cap.get(cv2.CAP_PROP_FPS):.0f} fps. '
                  'Calibrate and use the camera at the SAME resolution.')
        self.failures = 0
        self.ended = False

    def read(self):
        ok, frame = self.cap.read()
        # End of a video file, or the camera went away.
        self.failures = 0 if ok else self.failures + 1
        if self.failures > 50:
            self.ended = True
        return frame if ok else None

    def close(self):
        self.cap.release()


def open_source(args, raw=False):
    if args.source == 'zed':
        return ZedSource(args.resolution, args.fps, raw=raw)
    return OpenCvSource(args.source, args.width, args.height, args.fourcc)


def add_source_args(p):
    g = p.add_argument_group('camera')
    g.add_argument('--source', default='/dev/video0',
                   help='/dev/videoN or N (webcam, e.g. the Brio 100), "zed" (pyzed), '
                        'or a video URL/file (default /dev/video0)')
    g.add_argument('--resolution', default='HD720',
                   help='ZED resolution: HD2K, HD1080, HD1200, HD720, SVGA, VGA (default HD720)')
    g.add_argument('--fps', type=int, default=30, help='ZED fps (default 30)')
    g.add_argument('--width', type=int, default=1280, help='webcam width (default 1280)')
    g.add_argument('--height', type=int, default=720, help='webcam height (default 720)')
    g.add_argument('--fourcc', default='MJPG', help='webcam pixel format (default MJPG; "" = driver default)')


# ---------------------------------------------------------------------------
# Calibration file (ROS camera_info YAML)
# ---------------------------------------------------------------------------

def save_calibration(path, k, d, size, camera_name, extra=None):
    w, h = size
    p = np.hstack([k, np.zeros((3, 1))])
    data = {
        'image_width': int(w),
        'image_height': int(h),
        'camera_name': camera_name,
        'camera_matrix': {'rows': 3, 'cols': 3, 'data': [float(v) for v in k.flatten()]},
        'distortion_model': 'plumb_bob',
        'distortion_coefficients': {'rows': 1, 'cols': len(d), 'data': [float(v) for v in np.ravel(d)]},
        'rectification_matrix': {'rows': 3, 'cols': 3, 'data': [float(v) for v in np.eye(3).flatten()]},
        'projection_matrix': {'rows': 3, 'cols': 4, 'data': [float(v) for v in p.flatten()]},
    }
    if extra:
        data['calibration_info'] = extra
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, 'w') as f:
        yaml.safe_dump(data, f, sort_keys=False)
    print(f'Saved {path}')


def load_calibration(path):
    with open(path) as f:
        c = yaml.safe_load(f)
    k = np.array(c['camera_matrix']['data'], dtype=np.float64).reshape(3, 3)
    d = np.array(c['distortion_coefficients']['data'], dtype=np.float64)
    return k, d, (c['image_width'], c['image_height'])


def describe(k, d, size):
    w, h = size
    hfov = math.degrees(2 * math.atan(w / (2 * k[0, 0])))
    vfov = math.degrees(2 * math.atan(h / (2 * k[1, 1])))
    print(f'  image {w}x{h}, fx {k[0, 0]:.1f} fy {k[1, 1]:.1f}, cx {k[0, 2]:.1f} cy {k[1, 2]:.1f}, '
          f'FOV {hfov:.1f} x {vfov:.1f} deg')
    print(f'  distortion {np.round(np.ravel(d), 5).tolist()}')


# ---------------------------------------------------------------------------
# Subcommands
# ---------------------------------------------------------------------------

def cmd_board(args):
    board = make_board(args)
    # Pixel size chosen so the board prints at the requested physical size
    # on A4/Letter at 300 dpi; the real size still has to be measured.
    px_per_m = args.dpi / 0.0254
    size = (int(args.squares_x * args.square_length * px_per_m),
            int(args.squares_y * args.square_length * px_per_m))
    margin = int(0.01 * px_per_m)
    img = board.generateImage((size[0] + 2 * margin, size[1] + 2 * margin), marginSize=margin)
    cv2.imwrite(args.output, img)
    print(f'Wrote {args.output} ({img.shape[1]}x{img.shape[0]} px). Print at {args.dpi} dpi, '
          f'100% scale ("actual size"), on stiff flat card.')
    print(f'Intended: {args.squares_x}x{args.squares_y} squares of {args.square_length * 1000:.0f} mm, '
          f'markers {args.board_marker_length * 1000:.0f} mm, {args.board_dict}.')
    print('After printing, MEASURE a square and a marker and pass those values to calibrate.')


# Dictionaries tried by `identify`, smallest first within each marker size so
# the most specific one that contains every seen id is reported.
IDENTIFY_DICTS = [f'DICT_{b}X{b}_{n}' for b in (4, 5, 6, 7) for n in (50, 100, 250, 1000)] + \
    ['DICT_ARUCO_ORIGINAL', 'DICT_APRILTAG_16h5', 'DICT_APRILTAG_25h9',
     'DICT_APRILTAG_36h10', 'DICT_APRILTAG_36h11']


def identify_from_frame(gray):
    """Return the best guess of the board layout seen in `gray`, or None."""
    best = None
    for name in IDENTIFY_DICTS:
        if not hasattr(cv2.aruco, name):
            continue
        dictionary = get_dictionary(name)
        # Sub-pixel corners: plain detection shrinks the dark squares by about
        # a pixel under blur, which biases the marker/square ratio low.
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        corners, ids, _ = cv2.aruco.ArucoDetector(dictionary, params).detectMarkers(gray)
        if ids is None or len(ids) < 4:
            continue
        # Same-family dictionaries share their first codes, so keep the first
        # (smallest) one that sees the most markers.
        if best is None or len(ids) > len(best['ids']):
            best = {'dict': name, 'dictionary': dictionary, 'corners': corners, 'ids': ids}
    if best is None:
        return None

    n_markers = int(best['ids'].max()) + 1
    # A ChArUco board of X x Y squares has floor(X*Y/2) markers, ids 0..n-1.
    # Try every grid that could hold the seen ids, both layouts, and a range
    # of marker/square ratios; the right one makes the detected marker
    # corners fit the board plane with (near) zero homography error.
    results = []
    for x in range(3, 25):
        for y in range(3, 25):
            if (x * y) // 2 < n_markers or (x * y) // 2 > n_markers + 8:
                continue
            for legacy in (False, True):
                for ratio in np.arange(0.40, 0.951, 0.01):
                    board = cv2.aruco.CharucoBoard((x, y), 1.0, float(ratio), best['dictionary'])
                    board.setLegacyPattern(legacy)
                    # Marker corners on the board plane, by id (ChArUco's own
                    # matchImagePoints wants chessboard corners instead).
                    board_obj = dict(zip(np.ravel(board.getIds()).tolist(), board.getObjPoints()))
                    pairs = [(board_obj[i], c.reshape(4, 2))
                             for i, c in zip(best['ids'].ravel().tolist(), best['corners']) if i in board_obj]
                    if len(pairs) < 3:
                        continue
                    obj = np.concatenate([np.asarray(o, np.float32).reshape(4, 3) for o, _ in pairs])
                    img = np.concatenate([c for _, c in pairs]).astype(np.float32)
                    h, _ = cv2.findHomography(obj.reshape(-1, 3)[:, :2], img.reshape(-1, 2))
                    if h is None:
                        continue
                    proj = cv2.perspectiveTransform(obj.reshape(-1, 1, 3)[:, :, :2].astype(np.float32), h)
                    err = float(np.sqrt(np.mean(np.sum((proj - img.reshape(-1, 1, 2)) ** 2, axis=2))))
                    results.append((err, x, y, legacy, round(float(ratio), 2)))
    if not results:
        return None
    results.sort()
    err, x, y, legacy, ratio = results[0]
    # Prefer the exact marker count among near-equal fits (a bigger grid with
    # the same seen markers fits just as well).
    for r in results:
        if r[0] <= err * 1.05 + 0.05 and (r[1] * r[2]) // 2 == n_markers:
            err, x, y, legacy, ratio = r
            break
    return {'dict': best['dict'], 'markers_seen': len(best['ids']), 'max_id': n_markers - 1,
            'squares_x': x, 'squares_y': y, 'legacy': legacy, 'ratio': ratio, 'fit_px': err}


def cmd_identify(args):
    frames = []
    if args.images:
        frames = [cv2.imread(p, cv2.IMREAD_GRAYSCALE) for p in sorted(glob.glob(args.images))]
        frames = [f for f in frames if f is not None]
    else:
        source = open_source(args)
        print('Hold the whole board flat in front of the camera...')
    guesses = []
    try:
        deadline = time.monotonic() + args.timeout
        while len(guesses) < 3 and (args.images or time.monotonic() < deadline):
            if args.images:
                if not frames:
                    break
                gray = frames.pop(0)
            else:
                frame = source.read()
                if frame is None:
                    if source.ended:
                        break
                    continue
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            g = identify_from_frame(gray)
            if g:
                guesses.append(g)
                print(f"  frame: {g['dict']}, {g['markers_seen']} markers, {g['squares_x']}x{g['squares_y']} "
                      f"squares, ratio {g['ratio']}, legacy {g['legacy']}, fit {g['fit_px']:.2f} px")
    finally:
        if not args.images:
            source.close()
    if not guesses:
        sys.exit('No ChArUco board identified - is the whole board in view, flat and well lit?')
    g = min(guesses, key=lambda r: r['fit_px'])
    print(f"\nBoard: {g['squares_x']} x {g['squares_y']} squares, {g['dict']}, marker/square ratio {g['ratio']}"
          f"{', LEGACY layout' if g['legacy'] else ''} (fit {g['fit_px']:.2f} px, ids 0-{g['max_id']}).")
    if g['fit_px'] > 2.0:
        print('WARNING: poor fit - hold the board flatter/closer and run identify again.')
    print('Now MEASURE one square with a ruler (metres) and run:\n')
    print(f"  python3 tools/aruco_camera_calibration.py calibrate --squares-x {g['squares_x']} "
          f"--squares-y {g['squares_y']} --board-dict {g['dict']}{' --legacy' if g['legacy'] else ''} \\\n"
          f"      --square-length <SQUARE_M> --board-marker-length <{g['ratio']} x SQUARE_M> -o <output.yaml>")
    print('\nThe ratio is estimated from the image (about +/-0.03). That is fine: ChArUco only uses the'
          '\nmarkers to find the chessboard corners, and the camera intrinsics do not depend on the board'
          '\nscale at all - a ruler-measured square size just keeps the reported board distances right.')


def cmd_zed_factory(args):
    zed = ZedSource(args.resolution, args.fps, raw=args.raw)
    k, d, size = zed.factory_calibration(raw=args.raw)
    zed.close()
    view = 'left raw (unrectified)' if args.raw else 'left rectified'
    print(f'ZED factory calibration, {view}:')
    describe(k, d, size)
    save_calibration(args.output, k, d, size, f'zed_left_{args.resolution}',
                     {'source': f'ZED SDK factory calibration ({view})',
                      'note': 'use with the matching ZED view and resolution'})


def board_detector(args):
    params = cv2.aruco.CharucoParameters()
    return cv2.aruco.CharucoDetector(make_board(args), params)


def detect_board(detector, board, gray):
    corners, ids, marker_corners, marker_ids = detector.detectBoard(gray)
    if ids is None or len(ids) < 6:
        return None, None, marker_corners, marker_ids
    obj, img = board.matchImagePoints(corners, ids)
    return obj, img, marker_corners, marker_ids


def view_signature(img_pts, size):
    """Board centre (normalised), apparent size and tilt - to skip near-duplicate views."""
    pts = img_pts.reshape(-1, 2)
    w, h = size
    centre = pts.mean(axis=0) / np.array([w, h])
    span = np.ptp(pts, axis=0) / np.array([w, h])
    return np.array([centre[0], centre[1], span[0], span[1], span[0] / max(span[1], 1e-6) / 4])


def run_calibration(obj_points, img_points, size, args):
    flags = 0
    if args.fix_k3:
        flags |= cv2.CALIB_FIX_K3
    rms, k, d, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, size, None, None, flags=flags)
    errors = []
    for obj, img, r, t in zip(obj_points, img_points, rvecs, tvecs):
        proj, _ = cv2.projectPoints(obj, r, t, k, d)
        errors.append(float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - img.reshape(-1, 2)) ** 2, axis=1)))))
    return rms, k, d, errors


def calibrate_and_save(obj_points, img_points, size, args):
    if len(obj_points) < args.min_views:
        sys.exit(f'Only {len(obj_points)} usable views; need at least {args.min_views}.')
    rms, k, d, errors = run_calibration(obj_points, img_points, size, args)
    # One pass of outlier rejection: drop views much worse than the rest
    # (motion blur, a mis-detected corner) and recalibrate.
    cutoff = max(3 * float(np.median(errors)), 1.0)
    keep = [i for i, e in enumerate(errors) if e <= cutoff]
    if len(keep) < len(errors) and len(keep) >= args.min_views:
        print(f'Dropping {len(errors) - len(keep)} outlier view(s) (> {cutoff:.2f} px) and recalibrating')
        obj_points = [obj_points[i] for i in keep]
        img_points = [img_points[i] for i in keep]
        rms, k, d, errors = run_calibration(obj_points, img_points, size, args)

    print(f'\nCalibration from {len(obj_points)} views: RMS reprojection error {rms:.3f} px')
    describe(k, d, size)
    verdict = ('good' if rms < 0.5 else 'acceptable' if rms < 1.0
               else 'poor - check the board is flat and the sizes are measured')
    print(f'  quality: {verdict}')
    save_calibration(args.output, k, d, size, args.camera_name,
                     {'source': 'ChArUco calibration (aruco_camera_calibration.py)',
                      'rms_px': round(float(rms), 4), 'views': len(obj_points),
                      'board': f'{args.squares_x}x{args.squares_y} {args.board_dict} '
                               f'square {args.square_length} m marker {args.board_marker_length} m'})


def cmd_calibrate(args):
    board = make_board(args)
    detector = board_detector(args)
    obj_points, img_points, size = [], [], None

    if args.images:
        paths = sorted(glob.glob(args.images))
        if not paths:
            sys.exit(f'No images match {args.images!r}')
        for path in paths:
            gray = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
            if gray is None:
                continue
            size = (gray.shape[1], gray.shape[0])
            obj, img, _, _ = detect_board(detector, board, gray)
            if obj is not None and len(obj) >= args.min_corners:
                obj_points.append(obj)
                img_points.append(img)
        print(f'{len(obj_points)}/{len(paths)} images had a usable board')
        calibrate_and_save(obj_points, img_points, size, args)
        return

    source = open_source(args, raw=args.raw)
    if args.save_frames:
        os.makedirs(args.save_frames, exist_ok=True)
    signatures, last_capture = [], 0.0
    print(f'Show the board at different distances, positions and tilts '
          f'(target {args.views} views). '
          + ('Ctrl+C to stop early.' if args.headless else
             'SPACE = capture now, C = calibrate, Q = quit.'))
    try:
        while len(obj_points) < args.views:
            frame = source.read()
            if frame is None:
                if source.ended:
                    print('No more frames from the source')
                    break
                continue
            size = (frame.shape[1], frame.shape[0])
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            obj, img, marker_corners, marker_ids = detect_board(detector, board, gray)
            usable = obj is not None and len(obj) >= args.min_corners
            capture = False
            if usable:
                sig = view_signature(img, size)
                novel = all(np.linalg.norm(sig - s) > args.novelty for s in signatures)
                capture = novel and time.monotonic() - last_capture > args.interval

            key = -1
            if not args.headless:
                show = frame.copy()
                if marker_ids is not None:
                    cv2.aruco.drawDetectedMarkers(show, marker_corners, marker_ids)
                if usable:
                    for p in img.reshape(-1, 2):
                        cv2.circle(show, (int(p[0]), int(p[1])), 4, (0, 255, 0), -1)
                for s in signatures:  # where views have been taken
                    cv2.circle(show, (int(s[0] * size[0]), int(s[1] * size[1])), 6, (255, 128, 0), 2)
                status = (f'views {len(obj_points)}/{args.views}  corners '
                          f'{0 if obj is None else len(obj)}  ' + ('NEW VIEW' if capture else
                          'move/tilt the board' if usable else 'board not seen'))
                cv2.putText(show, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                cv2.imshow('ChArUco calibration', show)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    sys.exit('Quit without calibrating')
                if key == ord('c'):
                    break
                if key == ord(' ') and usable:
                    capture = True

            if capture:
                obj_points.append(obj)
                img_points.append(img)
                signatures.append(view_signature(img, size))
                last_capture = time.monotonic()
                if args.save_frames:
                    cv2.imwrite(os.path.join(args.save_frames, f'view_{len(obj_points):03d}.png'), frame)
                print(f'  captured view {len(obj_points)}/{args.views} ({len(obj)} corners)')
    except KeyboardInterrupt:
        print('\nStopped - calibrating with the views captured so far')
    finally:
        source.close()
        if not args.headless:
            cv2.destroyAllWindows()
    calibrate_and_save(obj_points, img_points, size, args)


def _band_mean(gray, h_page, depth):
    """Mean brightness of a thin band `depth` (fraction of the side) inside
    every edge of the square that h_page maps from the unit square."""
    t = np.linspace(0.15, 0.85, 12)
    pts = np.concatenate([
        np.stack([t, np.full_like(t, depth)], 1), np.stack([t, np.full_like(t, 1 - depth)], 1),
        np.stack([np.full_like(t, depth), t], 1), np.stack([np.full_like(t, 1 - depth), t], 1)])
    img = cv2.perspectiveTransform(pts.reshape(-1, 1, 2).astype(np.float32), h_page).reshape(-1, 2)
    h, w = gray.shape
    xs = np.clip(np.round(img[:, 0]).astype(int), 0, w - 1)
    ys = np.clip(np.round(img[:, 1]).astype(int), 0, h - 1)
    return float(gray[ys, xs].mean())


def fix_page_edge(gray, quad, margin_ratio):
    """ADR markers are printed with a white margin on a page that's cut to a
    square. Against a darker background the detector can lock onto the PAGE
    edge instead of the marker, reading the marker ~10 % too big (distance
    ~10 % short). If the band just inside the detected square is white
    rather than the marker's dark border, step the corners in by the known
    margin and refine them on the real marker edge.

    margin_ratio = white border / page side (10 mm / 210 mm for the ADR).
    Returns (corners 4x2, corrected?)."""
    quad = quad.reshape(4, 2).astype(np.float32)
    if margin_ratio <= 0:
        return quad, False
    unit = np.float32([[0, 0], [1, 0], [1, 1], [0, 1]])
    h_quad = cv2.getPerspectiveTransform(unit, quad)
    # Correct detection: both bands fall in the marker's dark outer border
    # (1/6 of the marker). Page-edge detection: the outer band is white margin.
    outer = _band_mean(gray, h_quad, margin_ratio * 0.5)
    inner = _band_mean(gray, h_quad, margin_ratio + (1 - 2 * margin_ratio) / 12)
    if outer - inner < 40:
        return quad, False
    m = margin_ratio
    inner_quad = cv2.perspectiveTransform(
        np.float32([[m, m], [1 - m, m], [1 - m, 1 - m], [m, 1 - m]]).reshape(-1, 1, 2), h_quad).reshape(4, 2)
    side = float(np.linalg.norm(inner_quad[1] - inner_quad[0]))
    win = int(max(2, min(10, side * 0.04)))
    cv2.cornerSubPix(gray, inner_quad.reshape(-1, 1, 2), (win, win), (-1, -1),
                     (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01))
    return inner_quad.reshape(4, 2), True


def parse_ids(spec):
    """'420-424' or '420,421,423' -> set of ints; '' -> empty set (all ids)."""
    ids = set()
    for part in filter(None, (p.strip() for p in (spec or '').split(','))):
        if '-' in part:
            lo, hi = part.split('-')
            ids.update(range(int(lo), int(hi) + 1))
        else:
            ids.add(int(part))
    return ids


def cmd_verify(args):
    k, d, calib_size = load_calibration(args.calib)
    source = open_source(args, raw=args.raw)
    detector = cv2.aruco.ArucoDetector(get_dictionary(args.dict), cv2.aruco.DetectorParameters())
    half = args.marker_length / 2
    # Marker corners in the marker's own frame (OpenCV order: TL, TR, BR, BL).
    marker_obj = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]],
                          dtype=np.float32)
    wanted = parse_ids(args.ids)
    page = args.marker_length + 2 * args.page_border
    margin_ratio = args.page_border / page if args.page_border > 0 else 0.0
    warned = False
    print(f'Looking for {args.dict} markers of {args.marker_length * 100:.1f} cm'
          + (f', ids {args.ids}' if wanted else '')
          + (f', {args.page_border * 1000:.0f} mm white page border' if margin_ratio else '') + '. '
          + ('Ctrl+C to stop.' if args.headless else 'Q to quit.'))
    last_print = 0.0
    try:
        while True:
            frame = source.read()
            if frame is None:
                if source.ended:
                    print('No more frames from the source')
                    break
                continue
            if (frame.shape[1], frame.shape[0]) != tuple(calib_size) and not warned:
                print(f'WARNING: frames are {frame.shape[1]}x{frame.shape[0]} but the calibration is '
                      f'{calib_size[0]}x{calib_size[1]} - distances will be wrong')
                warned = True
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)
            lines = []
            if ids is not None:
                keep = [j for j, i in enumerate(ids.ravel()) if not wanted or i in wanted]
                corners = [corners[j] for j in keep]
                ids = ids[keep] if keep else None
            if ids is not None:
                fixed_corners = []
                for c in corners:
                    q, corrected = fix_page_edge(gray, c, margin_ratio)
                    fixed_corners.append(q.reshape(1, 4, 2))
                    if corrected:
                        cv2.polylines(frame, [c.reshape(4, 2).astype(np.int32)], True, (0, 0, 255), 1)
                corners = fixed_corners
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                for c, i in zip(corners, ids.ravel()):
                    ok, rvec, tvec = cv2.solvePnP(marker_obj, c.reshape(4, 2), k, d,
                                                  flags=cv2.SOLVEPNP_IPPE_SQUARE)
                    if not ok:
                        continue
                    cv2.drawFrameAxes(frame, k, d, rvec, tvec, args.marker_length * 0.75)
                    x, y, z = tvec.ravel()  # camera frame: x right, y down, z forward
                    dist = float(np.linalg.norm(tvec))
                    bearing = math.degrees(math.atan2(x, z))  # + = marker to the right
                    lines.append(f'id {i}: {dist:.2f} m, bearing {bearing:+.1f} deg '
                                 f'(x {x:+.2f} y {y:+.2f} z {z:+.2f})')
                    cv2.putText(frame, f'{i}: {dist:.2f} m {bearing:+.0f} deg',
                                tuple(int(v) for v in c.reshape(4, 2)[0]),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            if time.monotonic() - last_print > 0.5:
                last_print = time.monotonic()
                print(' | '.join(lines) if lines else 'no markers')
            if not args.headless:
                cv2.imshow('ArUco verify', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        source.close()
        if not args.headless:
            cv2.destroyAllWindows()


# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='cmd', required=True)

    p = sub.add_parser('board', help='write a printable ChArUco board')
    add_board_args(p)
    p.add_argument('--dpi', type=int, default=300)
    p.add_argument('-o', '--output', default='charuco_board.png')
    p.set_defaults(func=cmd_board)

    p = sub.add_parser('identify', help="work out an existing ChArUco board's layout")
    add_source_args(p)
    p.add_argument('--images', help="identify from saved images instead, e.g. 'frames/*.png'")
    p.add_argument('--timeout', type=float, default=60.0, help='seconds to look for the board (default 60)')
    p.set_defaults(func=cmd_identify, raw=False)

    p = sub.add_parser('zed-factory', help="save the ZED's factory calibration")
    p.add_argument('--resolution', default='HD720')
    p.add_argument('--fps', type=int, default=30)
    p.add_argument('--raw', action='store_true', help='unrectified intrinsics + distortion instead')
    p.add_argument('-o', '--output', default='zed_left_calibration.yaml')
    p.set_defaults(func=cmd_zed_factory)

    p = sub.add_parser('calibrate', help='calibrate from a ChArUco board')
    add_source_args(p)
    add_board_args(p)
    p.add_argument('--images', help="calibrate from saved images instead, e.g. 'frames/*.png'")
    p.add_argument('--raw', action='store_true', help='ZED: use unrectified images')
    p.add_argument('--views', type=int, default=25, help='views to capture (default 25)')
    p.add_argument('--min-views', type=int, default=10)
    p.add_argument('--min-corners', type=int, default=12, help='ChArUco corners needed per view')
    p.add_argument('--novelty', type=float, default=0.08,
                   help='how different a view must be from earlier ones (default 0.08)')
    p.add_argument('--interval', type=float, default=0.7, help='min seconds between auto captures')
    p.add_argument('--fix-k3', action='store_true', help='fix k3=0 (helps with few/central views)')
    p.add_argument('--headless', action='store_true', help='no preview window (e.g. over SSH)')
    p.add_argument('--save-frames', help='directory to save captured frames to')
    p.add_argument('--camera-name', default='camera')
    p.add_argument('-o', '--output', default='camera_calibration.yaml')
    p.set_defaults(func=cmd_calibrate)

    p = sub.add_parser('verify', help='live ArUco distance/bearing with a calibration')
    add_source_args(p)
    p.add_argument('--calib', required=True, help='calibration YAML from zed-factory or calibrate')
    p.add_argument('--raw', action='store_true', help='ZED: use unrectified images (match the calibration)')
    # Defaults = the ADR markers from aruco_generate.py: DICT_4X4_1000, ids
    # 420-424, 190 mm marker with a 10 mm white border, page cut to 210 mm.
    p.add_argument('--dict', default='DICT_4X4_1000', help="markers' dictionary (default DICT_4X4_1000, the ADR's)")
    p.add_argument('--ids', default='420-424', help="marker ids to use, e.g. '420-424' or '420,422'; '' = any")
    p.add_argument('--marker-length', type=float, default=0.19,
                   help='MEASURED outer side of the coloured marker square, metres (default 0.19, the ADR)')
    p.add_argument('--page-border', type=float, default=0.01,
                   help='white border round the marker on the cut page, metres (default 0.01; 0 = off)')
    p.add_argument('--headless', action='store_true')
    p.set_defaults(func=cmd_verify)

    args = parser.parse_args()
    args.func(args)


if __name__ == '__main__':
    main()
