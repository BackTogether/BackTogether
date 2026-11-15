#!/usr/bin/env python3
"""Full screen animation on top of everything: covers the terminals and rviz with the robot
face gifs in assets/, so the robot shows one clean screen during the demo.

A borderless fullscreen always-on-top window (Tk) playing the gifs one after another in a
random order (every gif is played once before any of them repeats), each one centred on a
plain background. It keeps raising itself (rviz opens after it and would otherwise end up in
front) and can be hidden and shown again at runtime without restarting the node.

Params (set in bringup.launch):
  ~images       comma separated gif / image paths, or directories holding them
                (default: every gif in bringup/assets, played in random order)
  ~loops        times each gif is played before switching to the next one (default 1)
  ~fit          contain: whole frame, screen filled with ~background around it (default)
                cover:   fill the screen, crop what does not fit
                stretch: fill the screen, ignore the aspect ratio
                none:    original size, centred
  ~background   colour behind the animation (default black)
  ~delay        seconds to wait before showing it, so it opens after rviz (default 3.0)
  ~topmost      keep it above every other window (default true)
  ~raise_period seconds between two self-raises, 0 to never re-raise (default 2.0)
  ~hide_cursor  hide the mouse pointer over the animation (default true)

Topics:
  /splash/show (std_msgs/Bool)  true shows it again, false hides it (the node stays up):
                                rostopic pub -1 /splash/show std_msgs/Bool "data: false"

Keys (with the window focused): Esc or q closes it, h hides it, t toggles always-on-top,
n skips to the next gif.
Closing it only ends this node; the rest of the launch keeps running.
"""


import os
import random
import sys

import rospy
from std_msgs.msg import Bool

DEFAULT_FIT = "contain"
DEFAULT_BACKGROUND = "black"
DEFAULT_DELAY = 3.0
DEFAULT_RAISE_PERIOD = 2.0
DEFAULT_LOOPS = 1
FALLBACK_FRAME_MS = 80  # frame delay when the gif does not carry one
MIN_FRAME_MS = 20  # Tk cannot keep up with faster frames
STILL_MS = 3000  # how long a still image is held before the next one
TICK_MS = 100  # how often the Tk loop checks for shutdown / show-hide requests


def asset_dir():
    """bringup/assets: from the installed share/ directory when the package is installed,
    else next to this script in the source tree."""
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(here, os.pardir, "assets"),
        os.path.join(here, "assets"),
    ]
    try:
        import rospkg

        candidates.insert(0, os.path.join(rospkg.RosPack().get_path("bringup"), "assets"))
    except Exception:
        pass
    for path in candidates:
        path = os.path.normpath(path)
        if os.path.isdir(path):
            return path
    return os.path.normpath(candidates[-1])


def gifs_in(directory):
    """Every gif of the directory, sorted by name (the face gifs are numbered)."""
    try:
        names = sorted(os.listdir(directory))
    except OSError:
        return []
    return [
        os.path.join(directory, name) for name in names if name.lower().endswith(".gif")
    ]


def resolve_paths(spec):
    """The ~images parameter as a list of existing files. Entries are files (any image) or
    directories (their gifs); an empty parameter means every gif in assets/."""
    entries = [entry.strip() for entry in str(spec).split(",") if entry.strip()]
    if not entries:
        return gifs_in(asset_dir())
    paths = []
    for entry in entries:
        entry = os.path.normpath(os.path.expanduser(entry))
        if os.path.isdir(entry):
            paths.extend(gifs_in(entry))
        elif os.path.isfile(entry):
            paths.append(entry)
        else:
            rospy.logwarn("splash: no image at %s, skipping it", entry)
    return paths


def scaled_size(image_w, image_h, screen_w, screen_h, fit):
    """Size to draw a frame at on a screen_w x screen_h screen."""
    if fit == "stretch":
        return screen_w, screen_h
    if fit == "none":
        return image_w, image_h
    scale_w = float(screen_w) / image_w
    scale_h = float(screen_h) / image_h
    scale = max(scale_w, scale_h) if fit == "cover" else min(scale_w, scale_h)
    return max(1, int(round(image_w * scale))), max(1, int(round(image_h * scale)))


def frame_delay(raw_duration):
    """Frame duration in ms, as fast as Tk can actually play it."""
    try:
        duration = int(raw_duration)
    except (TypeError, ValueError):
        duration = 0
    return max(MIN_FRAME_MS, duration or FALLBACK_FRAME_MS)


def load_frames_tk(path, screen_w, screen_h, fit):
    """Frames read with Tk's own gif / png reader, used when Pillow is missing: whole number
    zoom or subsample only, and no per frame duration."""
    import tkinter as tk

    frames = []
    zoom, shrink = 1, 1
    index = 0
    while True:
        try:
            photo = tk.PhotoImage(file=path, format="gif -index %d" % index)
        except tk.TclError:
            break
        if index == 0:
            width, height = photo.width(), photo.height()
            target_w, target_h = scaled_size(width, height, screen_w, screen_h, fit)
            # Tk can only scale by whole factors: round towards filling the screen rather
            # than towards a much smaller image (a 4% overshoot is clipped, halving it
            # would not be).
            zoom = max(1, min(target_w // width, target_h // height))
            shrink = max(1, max(width // target_w, height // target_h))
        if zoom > 1:
            photo = photo.zoom(zoom, zoom)
        elif shrink > 1:
            photo = photo.subsample(shrink, shrink)
        frames.append((photo, FALLBACK_FRAME_MS))
        index += 1
    if not frames:  # not a gif: a still image
        frames.append((tk.PhotoImage(file=path), STILL_MS))
    return frames


def load_frames(path, screen_w, screen_h, fit, background):
    """The gif as a list of (Tk image, duration in ms), every frame scaled for the screen.
    Pillow when it is installed (smooth scaling, real frame durations), else Tk's reader."""
    try:
        from PIL import Image, ImageColor, ImageSequence, ImageTk
    except ImportError:
        rospy.logwarn("splash: Pillow not installed, falling back to Tk's gif reader "
                      "(coarser scaling, fixed frame rate). Install it with: "
                      "sudo apt install python3-pil.imagetk")
        return load_frames_tk(path, screen_w, screen_h, fit)

    try:
        back_rgb = ImageColor.getrgb(str(background))
    except ValueError:
        back_rgb = (0, 0, 0)

    frames = []
    with Image.open(path) as animation:  # every frame is copied out, so it can be closed
        target_w, target_h = scaled_size(
            animation.width, animation.height, screen_w, screen_h, fit
        )
        for frame in ImageSequence.Iterator(animation):
            # gif frames can be partly transparent (only what changed is stored): flatten each
            # one on the background colour so the frame is complete on its own
            flat = Image.new("RGB", frame.size, back_rgb)
            rgba = frame.convert("RGBA")
            flat.paste(rgba, (0, 0), rgba)
            if (target_w, target_h) != flat.size:
                # the faces are pixel art: blown up they stay crisp with NEAREST, shrunk they
                # need LANCZOS to average the pixels away
                resample = Image.NEAREST if target_w >= flat.width else Image.LANCZOS
                flat = flat.resize((target_w, target_h), resample)
            frames.append((ImageTk.PhotoImage(flat), frame_delay(frame.info.get("duration"))))
    if len(frames) == 1:  # a still image, not an animation
        frames = [(frames[0][0], STILL_MS)]
    return frames


class Splash(object):
    def __init__(self, paths, fit, background, topmost, raise_period, hide_cursor, loops):
        import tkinter as tk

        self.tk = tk
        self.paths = paths
        self.fit = fit
        self.background = background
        self.topmost = topmost
        self.raise_period = raise_period
        self.loops = loops
        self.visible = True
        self.last_raise = rospy.get_time()
        self.show_request = None  # set by the /splash/show callback, applied in the Tk loop

        self.clips = {}  # index in paths -> frames, filled the first time the gif is played
        self.queue = []  # gifs left in this random round
        self.current = -1
        self.frames = []
        self.frame_index = 0
        self.loops_left = 0
        self.frame_job = None

        self.root = tk.Tk()
        self.root.title("bobac3 splash")
        self.root.configure(bg=background)
        self.screen_w = self.root.winfo_screenwidth()
        self.screen_h = self.root.winfo_screenheight()

        self.label = tk.Label(self.root, bg=background, borderwidth=0, highlightthickness=0)
        self.label.place(relx=0.5, rely=0.5, anchor="center")  # centred whatever its size

        try:
            self.root.attributes("-fullscreen", True)
        except tk.TclError:  # window manager without fullscreen support
            self.root.overrideredirect(True)
            self.root.geometry("%dx%d+0+0" % (self.screen_w, self.screen_h))
        self.root.attributes("-topmost", bool(topmost))
        if hide_cursor:
            self.root.config(cursor="none")

        self.root.bind("<Escape>", lambda _event: self.close())
        self.root.bind("q", lambda _event: self.close())
        self.root.bind("h", lambda _event: self.set_visible(False))
        self.root.bind("t", lambda _event: self.toggle_topmost())
        self.root.bind("n", lambda _event: self.play_next())
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.focus_force()

        rospy.loginfo("splash: playing %d gif(s) in random order on %dx%d "
                      "(fit=%s, loops=%d, topmost=%s)",
                      len(paths), self.screen_w, self.screen_h, fit, loops, bool(topmost))
        self.play_next()

    def clip(self, index):
        """Frames of one gif, scaled once and kept in memory for the next rounds."""
        if index not in self.clips:
            self.clips[index] = load_frames(
                self.paths[index], self.screen_w, self.screen_h, self.fit, self.background
            )
            rospy.loginfo("splash: loaded %s (%d frames)",
                          self.paths[index], len(self.clips[index]))
        return self.clips[index]

    def next_index(self):
        """Next gif of the random round, reshuffled when the round runs out: every gif is
        played once per round, and a round never starts with the one just played."""
        if not self.queue:
            order = list(range(len(self.paths)))
            random.shuffle(order)
            if len(order) > 1 and order[0] == self.current:
                order.append(order.pop(0))
            self.queue = order
        return self.queue.pop(0)

    def play_next(self):
        """Start the next gif from its first frame."""
        self.cancel_frame_job()
        index = self.next_index()
        try:
            frames = self.clip(index)
        except Exception as error:  # one broken file must not stop the rotation
            rospy.logerr("splash: could not read %s (%s), dropping it",
                         self.paths[index], error)
            self.paths.pop(index)
            self.clips = {}  # the indexes moved
            self.queue = []
            self.current = -1
            if not self.paths:
                self.close()
                return
            self.play_next()
            return
        self.current = index
        self.frames = frames
        self.frame_index = 0
        self.loops_left = self.loops
        self.show_frame()

    def show_frame(self):
        """Draw the current frame and schedule the next one (or the next gif)."""
        photo, duration = self.frames[self.frame_index]
        self.label.configure(image=photo)
        self.label.image = photo  # keep a reference: Tk does not own the image
        if len(self.frames) == 1 and len(self.paths) == 1:
            return  # one still image and nothing to switch to
        self.frame_job = self.root.after(duration, self.advance)

    def advance(self):
        self.frame_job = None
        self.frame_index += 1
        if self.frame_index < len(self.frames):
            self.show_frame()
            return
        self.frame_index = 0
        self.loops_left -= 1
        if self.loops_left > 0 or len(self.paths) == 1:
            self.show_frame()
        else:
            self.play_next()

    def cancel_frame_job(self):
        if self.frame_job is not None:
            try:
                self.root.after_cancel(self.frame_job)
            except self.tk.TclError:
                pass
            self.frame_job = None

    def set_visible(self, visible):
        if visible == self.visible:
            return
        self.visible = visible
        if visible:
            self.root.deiconify()
            self.raise_window()
            self.play_next()  # the animation is paused while hidden
        else:
            self.cancel_frame_job()
            self.root.withdraw()
        rospy.loginfo("splash: %s", "shown" if visible else "hidden")

    def toggle_topmost(self):
        self.topmost = not self.topmost
        self.root.attributes("-topmost", self.topmost)
        rospy.loginfo("splash: topmost %s", "on" if self.topmost else "off")

    def raise_window(self):
        """rviz and the terminals open after this window, so put it back in front."""
        if not self.visible:
            return
        try:
            if self.topmost:
                self.root.attributes("-topmost", True)
            self.root.lift()
        except self.tk.TclError:
            pass

    def on_show(self, msg):
        self.show_request = bool(msg.data)  # applied in tick(): Tk is not thread safe

    def close(self):
        rospy.loginfo("splash: closing")
        self.cancel_frame_job()
        try:
            self.root.destroy()
        except self.tk.TclError:
            pass

    def tick(self):
        """Runs in the Tk main loop: shutdown, show / hide requests and the self-raise."""
        if rospy.is_shutdown():
            self.close()
            return
        if self.show_request is not None:
            request, self.show_request = self.show_request, None
            self.set_visible(request)
        if self.raise_period > 0:
            now = rospy.get_time()
            if now - self.last_raise >= self.raise_period:
                self.last_raise = now
                self.raise_window()
        self.root.after(TICK_MS, self.tick)

    def run(self):
        self.last_raise = rospy.get_time()
        self.root.after(TICK_MS, self.tick)
        self.root.mainloop()


def main():
    rospy.init_node("splash")

    images = rospy.get_param("~images", "")
    paths = resolve_paths(images)
    fit = str(rospy.get_param("~fit", DEFAULT_FIT)).lower()
    background = rospy.get_param("~background", DEFAULT_BACKGROUND)
    delay = float(rospy.get_param("~delay", DEFAULT_DELAY))
    topmost = bool(rospy.get_param("~topmost", True))
    raise_period = float(rospy.get_param("~raise_period", DEFAULT_RAISE_PERIOD))
    hide_cursor = bool(rospy.get_param("~hide_cursor", True))
    loops = max(1, int(rospy.get_param("~loops", DEFAULT_LOOPS)))

    if fit not in ("contain", "cover", "stretch", "none"):
        rospy.logwarn("splash: unknown fit '%s', using '%s'", fit, DEFAULT_FIT)
        fit = DEFAULT_FIT
    if not paths:
        rospy.logerr("splash: no gif to play (~images='%s'), not showing anything", images)
        return
    if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        rospy.logerr("splash: no DISPLAY / WAYLAND_DISPLAY set (no graphical session reachable "
                     "from here), not showing anything. Over ssh, start the launch on the "
                     "robot's own session, or export DISPLAY=:0 before roslaunch")
        return

    try:
        import tkinter  # noqa: F401  (imported here to give one clear message)
    except ImportError:
        rospy.logerr("splash: tkinter is missing, not showing anything. "
                     "Install it with: sudo apt install python3-tk")
        return

    if delay > 0:
        rospy.loginfo("splash: waiting %.1fs before covering the screen", delay)
        rospy.sleep(delay)
    if rospy.is_shutdown():
        return

    try:
        splash = Splash(paths, fit, background, topmost, raise_period, hide_cursor, loops)
    except Exception as error:  # a broken display must not take the rest of the launch down
        rospy.logerr("splash: could not open the window (%s), not showing anything", error)
        return

    rospy.Subscriber("/splash/show", Bool, splash.on_show, queue_size=1)
    splash.run()
    rospy.loginfo("splash: window closed")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        sys.exit(0)
