"""Is the person responsive? Checks on the head camera once the robot is next to them.

Runs ON THE ROBOT with its system Python 3.8 (OpenCV and numpy are already there).

The head camera can only be opened by one process. When the ROS camera node has it
(`roslaunch ar_pose usbcam_head.launch`) frames come from its topic; otherwise the device is
opened directly, so this works with and without ROS and never fights the node for the camera.

Try it on the robot, or on a laptop with a webcam or a recorded video:

    python3 responsiveness.py hand --seconds 8        # "raise your hand if you can hear me"
    python3 responsiveness.py moving --seconds 3
    python3 responsiveness.py standing
    python3 responsiveness.py photo --path out.jpg
    python3 responsiveness.py standing --camera 0            # laptop webcam
    python3 responsiveness.py moving --camera person.mp4     # recorded video

Pose needs MediaPipe, which is not on the robot yet. 0.10.9 has Python 3.8 wheels and, unlike
0.10.10 and 0.10.11, does not pull jax and torch. Install it without its dependencies so pip
does not replace the robot's OpenCV and numpy, then add the small ones it really imports:

    python3 -m pip install --user --no-deps mediapipe==0.10.9
    python3 -m pip install --user absl-py attrs flatbuffers "protobuf>=3.11,<4"

Without MediaPipe, is_moving and take_photo still work on the whole image; hand_raised and
is_standing answer False.
"""

import argparse
import logging
import math
import threading
import time

import cv2
import numpy as np

HEAD_CAMERA_DEVICE = "/dev/head_camera"
HEAD_CAMERA_TOPIC = "/head_camera/image_raw"
FIRST_FRAME_WAIT_S = 3.0
FRAME_POLL_S = 0.02
WARMUP_FRAMES = 5  # the first frames of a USB camera are dark
POSE_MODEL_COMPLEXITY = 1  # 0 is faster and less accurate, 2 is too slow for the robot CPU
MIN_VISIBILITY = 0.5
HAND_RAISE_MARGIN = 0.3  # wrist this fraction of the torso length above the shoulder
HAND_RAISED_FRAMES = 3  # in a row, so one bad pose estimate is not a yes
UPRIGHT_MAX_TILT_DEG = 35.0
STANDING_LOOK_S = 1.5
MOTION_SAMPLE_S = 0.25  # compare images this far apart: slow movements show up too
MOTION_BLUR_PX = 9  # hides camera noise
MOTION_PIXEL_CHANGE = 25  # grey levels (0-255) for a pixel to count as changed
MOTION_AREA = 0.01  # fraction of the body area that must change in one comparison
MOTION_SAMPLES = 2  # comparisons that must show movement
MOTION_BOX_MARGIN = 0.15
PHOTO_LOOK_S = 1.0
PHOTO_MARGIN = 0.25
# MediaPipe pose landmark indexes
LEFT_SHOULDER, RIGHT_SHOULDER = 11, 12
LEFT_WRIST, RIGHT_WRIST = 15, 16
LEFT_HIP, RIGHT_HIP = 23, 24
ROS_CHANNEL_ORDERS = {"rgb8": cv2.COLOR_RGB2BGR, "bgr8": None}

log = logging.getLogger("responsiveness")
camera = HEAD_CAMERA_DEVICE  # device path, webcam index or video file; the CLI can change it
_pose = None


class _DeviceFrames:
    """Frames straight from a camera device, a webcam index or a video file."""

    def __init__(self, source) -> None:
        self._capture = cv2.VideoCapture(source)
        is_live = not isinstance(source, str) or source.startswith("/dev/")
        for _ in range(WARMUP_FRAMES if is_live else 0):
            self._capture.read()

    def read(self):
        grabbed, frame = self._capture.read()
        return frame if grabbed else None

    def close(self) -> None:
        self._capture.release()


class _RosFrames:
    """Frames from the topic of the ROS node that owns the head camera."""

    def __init__(self, rospy, image_type) -> None:
        self._latest = None
        self._fresh = threading.Event()
        self._subscriber = rospy.Subscriber(
            HEAD_CAMERA_TOPIC, image_type, self._on_image, queue_size=1, buff_size=2**24
        )

    def _on_image(self, message) -> None:
        conversion = ROS_CHANNEL_ORDERS.get(message.encoding, "unsupported")
        if conversion == "unsupported":
            log.warning("unsupported camera encoding %s", message.encoding)
            return
        rows = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
        frame = rows[:, : message.width * 3].reshape(message.height, message.width, 3)
        self._latest = frame.copy() if conversion is None else cv2.cvtColor(frame, conversion)
        self._fresh.set()

    def read(self):
        """The next new frame; None if the node stops publishing."""
        if not self._fresh.wait(FIRST_FRAME_WAIT_S):
            return None
        self._fresh.clear()
        return self._latest

    def close(self) -> None:
        self._subscriber.unregister()


def _ros_camera_modules():
    """(rospy, Image) when the ROS camera node is publishing the head camera, else None."""
    if camera != HEAD_CAMERA_DEVICE:
        return None
    try:
        import rospy
        from sensor_msgs.msg import Image

        topics = [name for name, _ in rospy.get_published_topics()]
    except Exception:  # no ROS on this machine, or no roscore: use the device
        return None
    if HEAD_CAMERA_TOPIC not in topics:
        return None
    if not rospy.core.is_initialized():
        # Standalone use (CLI, care_server.py). Inside the bridge the node already exists.
        rospy.init_node("responsiveness", anonymous=True, disable_signals=True)
    return rospy, Image


def _open_frames():
    ros = _ros_camera_modules()
    if ros is not None:
        return _RosFrames(*ros)
    return _DeviceFrames(camera)


def _frames_for(seconds: float):
    """Yield (time, frame) for `seconds`; stops early if the camera gives no more frames."""
    frames = _open_frames()
    try:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            frame = frames.read()
            if frame is None:
                log.warning("no frame from the head camera")
                return
            yield time.monotonic(), frame
            time.sleep(FRAME_POLL_S)
    finally:
        frames.close()


class _Pose:
    """MediaPipe pose on BGR frames. Landmarks are (x, y, visibility) with x and y in pixels,
    y growing downwards; None when nobody is seen."""

    def __init__(self) -> None:
        import mediapipe  # heavy, and only on the robot after A1: load it on first use

        # static_image_mode: detect on every frame. The tracking mode keeps state between
        # frames, and one model serves separate checks whose scenes are unrelated.
        self._model = mediapipe.solutions.pose.Pose(
            static_image_mode=True, model_complexity=POSE_MODEL_COMPLEXITY
        )

    def landmarks(self, frame):
        found = self._model.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)).pose_landmarks
        if found is None:
            return None
        height, width = frame.shape[:2]
        return [(point.x * width, point.y * height, point.visibility) for point in found.landmark]


def _find_person(frame):
    """Pose landmarks of the person in the frame; None if nobody is seen or MediaPipe is not
    installed (then the checks that can do without it use the whole frame)."""
    global _pose
    if _pose is None:
        try:
            _pose = _Pose()
        except ImportError:
            log.warning("mediapipe is not installed: no pose")
            return None
    return _pose.landmarks(frame)


def _visible(landmarks, *indexes) -> bool:
    return all(landmarks[index][2] >= MIN_VISIBILITY for index in indexes)


def _middle(landmarks, first: int, second: int) -> tuple:
    return (
        (landmarks[first][0] + landmarks[second][0]) / 2.0,
        (landmarks[first][1] + landmarks[second][1]) / 2.0,
    )


def _torso_length(landmarks) -> float:
    shoulders = _middle(landmarks, LEFT_SHOULDER, RIGHT_SHOULDER)
    hips = _middle(landmarks, LEFT_HIP, RIGHT_HIP)
    return math.hypot(shoulders[0] - hips[0], shoulders[1] - hips[1])


def _hand_is_raised(landmarks) -> bool:
    """A wrist clearly above its own shoulder in the image (works standing, sitting or lying)."""
    margin = HAND_RAISE_MARGIN * _torso_length(landmarks)
    for wrist, shoulder in ((LEFT_WRIST, LEFT_SHOULDER), (RIGHT_WRIST, RIGHT_SHOULDER)):
        if not _visible(landmarks, wrist, shoulder):
            continue
        if landmarks[wrist][1] < landmarks[shoulder][1] - margin:
            return True
    return False


def _is_upright(landmarks) -> bool:
    """Torso close to vertical with the shoulders above the hips: standing or sitting."""
    if not _visible(landmarks, LEFT_SHOULDER, RIGHT_SHOULDER, LEFT_HIP, RIGHT_HIP):
        return False
    shoulders = _middle(landmarks, LEFT_SHOULDER, RIGHT_SHOULDER)
    hips = _middle(landmarks, LEFT_HIP, RIGHT_HIP)
    down = hips[1] - shoulders[1]
    sideways = abs(hips[0] - shoulders[0])
    return down > 0 and math.degrees(math.atan2(sideways, down)) < UPRIGHT_MAX_TILT_DEG


def _person_box(landmarks, frame, margin: float) -> tuple:
    """(left, top, right, bottom) in pixels around the visible landmarks; the whole frame when
    there is no person."""
    height, width = frame.shape[:2]
    points = [(x, y) for x, y, visibility in landmarks or [] if visibility >= MIN_VISIBILITY]
    if not points:
        return 0, 0, width, height
    xs, ys = zip(*points)
    pad = margin * max(max(xs) - min(xs), max(ys) - min(ys))
    return (
        int(max(min(xs) - pad, 0)),
        int(max(min(ys) - pad, 0)),
        int(min(max(xs) + pad, width)),
        int(min(max(ys) + pad, height)),
    )


def hand_raised(timeout_s: float = 8.0) -> bool:
    """Say "raise your hand if you can hear me", then call this: True as soon as a hand stays
    up for a few frames, False if it does not happen within timeout_s."""
    raised_in_a_row = 0
    for _, frame in _frames_for(timeout_s):
        landmarks = _find_person(frame)
        raised_in_a_row = raised_in_a_row + 1 if landmarks and _hand_is_raised(landmarks) else 0
        if raised_in_a_row >= HAND_RAISED_FRAMES:
            return True
    return False


def is_moving(seconds: float = 3.0) -> bool:
    """Does the person move during `seconds`? Compares images of the body area over time, so
    the robot must stand still meanwhile."""
    box = None
    previous = None
    previous_time = 0.0
    moving_samples = 0
    for now, frame in _frames_for(seconds):
        if previous is not None and now - previous_time < MOTION_SAMPLE_S:
            continue
        if box is None:
            box = _person_box(_find_person(frame), frame, MOTION_BOX_MARGIN)
        left, top, right, bottom = box
        gray = cv2.cvtColor(frame[top:bottom, left:right], cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (MOTION_BLUR_PX, MOTION_BLUR_PX), 0)
        if previous is not None:
            changed = cv2.absdiff(gray, previous) > MOTION_PIXEL_CHANGE
            moving_samples += changed.mean() > MOTION_AREA
        previous, previous_time = gray, now
    return moving_samples >= MOTION_SAMPLES


def is_standing() -> bool:
    """Has the person got up? True when most frames of a short look show an upright torso
    (standing or sitting), False when lying down or when nobody is seen."""
    upright = 0
    seen = 0
    for _, frame in _frames_for(STANDING_LOOK_S):
        landmarks = _find_person(frame)
        if landmarks is None:
            continue
        seen += 1
        upright += _is_upright(landmarks)
    return seen > 0 and upright > seen / 2.0


def take_photo(path: str) -> bool:
    """Save the sharpest frame of a short look, cropped around the person (the whole frame if
    nobody is seen). Same signature as the take_photo the care flow expects."""
    best = None
    best_sharpness = -1.0
    for _, frame in _frames_for(PHOTO_LOOK_S):
        sharpness = cv2.Laplacian(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), cv2.CV_64F).var()
        if sharpness > best_sharpness:
            best, best_sharpness = frame, sharpness
    if best is None:
        return False
    left, top, right, bottom = _person_box(_find_person(best), best, PHOTO_MARGIN)
    return bool(cv2.imwrite(path, best[top:bottom, left:right]))


def main() -> None:
    global camera
    parser = argparse.ArgumentParser(description="responsiveness checks on the head camera")
    parser.add_argument("check", choices=["hand", "moving", "standing", "photo"])
    parser.add_argument("--camera", default=HEAD_CAMERA_DEVICE, help="device, index or video")
    parser.add_argument("--seconds", type=float, default=5.0, help="for hand and moving")
    parser.add_argument("--path", default="photo.jpg", help="for photo")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(name)s: %(message)s")
    camera = int(args.camera) if args.camera.isdigit() else args.camera
    checks = {
        "hand": lambda: hand_raised(args.seconds),
        "moving": lambda: is_moving(args.seconds),
        "standing": is_standing,
        "photo": lambda: take_photo(args.path),
    }
    print(args.check, "->", checks[args.check]())


if __name__ == "__main__":
    main()
