#!/usr/bin/env python3
"""Full screen image on top of everything: covers the terminals and rviz with
assets/base_image.png, so the robot shows one clean picture during the demo.

A borderless fullscreen always-on-top window (Tk) holding the image. It keeps raising
itself (rviz opens after it and would otherwise end up in front) and can be hidden and
shown again at runtime without restarting the node.

Params (set in bringup.launch):
  ~image        path of the image to show (default: bringup/assets/base_image.png)
  ~fit          contain: whole image, screen filled with ~background around it (default)
                cover:   fill the screen, crop what does not fit
                stretch: fill the screen, ignore the aspect ratio
                none:    original size, centred
  ~background   colour behind the image (default black)
  ~delay        seconds to wait before showing it, so it opens after rviz (default 3.0)
  ~topmost      keep it above every other window (default true)
  ~raise_period seconds between two self-raises, 0 to never re-raise (default 2.0)
  ~hide_cursor  hide the mouse pointer over the image (default true)

Topics:
  /splash/show (std_msgs/Bool)  true shows it again, false hides it (the node stays up):
                                rostopic pub -1 /splash/show std_msgs/Bool "data: false"

Keys (with the window focused): Esc or q closes it, h hides it, t toggles always-on-top.
Closing it only ends this node; the rest of the launch keeps running.
"""


import os
import sys

import rospy
from std_msgs.msg import Bool

DEFAULT_FIT = "contain"
DEFAULT_BACKGROUND = "black"
DEFAULT_DELAY = 3.0
DEFAULT_RAISE_PERIOD = 2.0
TICK_MS = 100  # how often the Tk loop checks for shutdown / show-hide requests


def default_image_path():
    """assets/base_image.png of this package: from the installed share/ directory when the
    package is installed, else next to this script in the source tree."""
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(here, os.pardir, "assets", "base_image.png"),
        os.path.join(here, "assets", "base_image.png"),
    ]
    try:
        import rospkg

        candidates.insert(
            0, os.path.join(rospkg.RosPack().get_path("bringup"), "assets", "base_image.png")
        )
    except Exception:
        pass
    for path in candidates:
        path = os.path.normpath(path)
        if os.path.isfile(path):
            return path
    return os.path.normpath(candidates[-1])


def scaled_size(image_w, image_h, screen_w, screen_h, fit):
    """Size to draw the image at on a screen_w x screen_h screen."""
    if fit == "stretch":
        return screen_w, screen_h
    if fit == "none":
        return image_w, image_h
    scale_w = float(screen_w) / image_w
    scale_h = float(screen_h) / image_h
    scale = max(scale_w, scale_h) if fit == "cover" else min(scale_w, scale_h)
    return max(1, int(round(image_w * scale))), max(1, int(round(image_h * scale)))


def load_image(path, screen_w, screen_h, fit):
    """The image as a Tk-ready object, scaled for the screen. Pillow when it is installed
    (smooth scaling, any format), else Tk's own PNG reader with integer zoom / subsample."""
    try:
        from PIL import Image, ImageTk
    except ImportError:
        rospy.logwarn("splash: Pillow not installed, falling back to Tk's PNG reader "
                      "(coarser scaling). Install it with: sudo apt install python3-pil.imagetk")
        import tkinter as tk

        photo = tk.PhotoImage(file=path)
        width, height = photo.width(), photo.height()
        target_w, target_h = scaled_size(width, height, screen_w, screen_h, fit)
        # Tk can only scale by whole factors: round towards filling the screen rather than
        # towards a much smaller image (a 4% overshoot is clipped, halving it would not be).
        zoom = max(1, min(target_w // width, target_h // height))
        shrink = max(1, max(width // target_w, height // target_h))
        if zoom > 1:
            photo = photo.zoom(zoom, zoom)
        elif shrink > 1:
            photo = photo.subsample(shrink, shrink)
        return photo

    image = Image.open(path)
    if image.mode not in ("RGB", "RGBA"):
        image = image.convert("RGB")
    target_w, target_h = scaled_size(image.width, image.height, screen_w, screen_h, fit)
    if (target_w, target_h) != (image.width, image.height):
        image = image.resize((target_w, target_h), Image.LANCZOS)
    return ImageTk.PhotoImage(image)


class Splash(object):
    def __init__(self, path, fit, background, topmost, raise_period, hide_cursor):
        import tkinter as tk

        self.tk = tk
        self.topmost = topmost
        self.raise_period = raise_period
        self.visible = True
        self.last_raise = rospy.get_time()
        self.show_request = None  # set by the /splash/show callback, applied in the Tk loop

        self.root = tk.Tk()
        self.root.title("bobac3 splash")
        self.root.configure(bg=background)
        screen_w = self.root.winfo_screenwidth()
        screen_h = self.root.winfo_screenheight()

        self.image = load_image(path, screen_w, screen_h, fit)

        label = tk.Label(self.root, image=self.image, bg=background, borderwidth=0,
                         highlightthickness=0)
        label.pack(expand=True)

        try:
            self.root.attributes("-fullscreen", True)
        except tk.TclError:  # window manager without fullscreen support
            self.root.overrideredirect(True)
            self.root.geometry("%dx%d+0+0" % (screen_w, screen_h))
        self.root.attributes("-topmost", bool(topmost))
        if hide_cursor:
            self.root.config(cursor="none")

        self.root.bind("<Escape>", lambda _event: self.close())
        self.root.bind("q", lambda _event: self.close())
        self.root.bind("h", lambda _event: self.set_visible(False))
        self.root.bind("t", lambda _event: self.toggle_topmost())
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.focus_force()

        rospy.loginfo("splash: showing %s on %dx%d (fit=%s, topmost=%s)",
                      path, screen_w, screen_h, fit, bool(topmost))

    def set_visible(self, visible):
        if visible == self.visible:
            return
        self.visible = visible
        if visible:
            self.root.deiconify()
            self.raise_window()
        else:
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

    path = rospy.get_param("~image", "") or default_image_path()
    path = os.path.expanduser(str(path))
    fit = str(rospy.get_param("~fit", DEFAULT_FIT)).lower()
    background = rospy.get_param("~background", DEFAULT_BACKGROUND)
    delay = float(rospy.get_param("~delay", DEFAULT_DELAY))
    topmost = bool(rospy.get_param("~topmost", True))
    raise_period = float(rospy.get_param("~raise_period", DEFAULT_RAISE_PERIOD))
    hide_cursor = bool(rospy.get_param("~hide_cursor", True))

    if fit not in ("contain", "cover", "stretch", "none"):
        rospy.logwarn("splash: unknown fit '%s', using '%s'", fit, DEFAULT_FIT)
        fit = DEFAULT_FIT
    if not os.path.isfile(path):
        rospy.logerr("splash: no image at %s, not showing anything", path)
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
        splash = Splash(path, fit, background, topmost, raise_period, hide_cursor)
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
