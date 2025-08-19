#!/usr/bin/env python3
"""
Event recorder for v2e output.
Press
  s – start/stop recording
  q – quit (writes everything to disk)
"""
from pathlib import Path
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from event_camera_msgs.msg import EventPacket
from cv_bridge import CvBridge

# fast C++ decoder for EventPacket
try:
    import event_camera_py as ecp          # sudo apt install ros-humble-event-camera-py
except ImportError as e:                   # or pip install event_camera_py
    raise RuntimeError("event_camera_py not found – install it!") from e

class EventRecorder(Node):
    BLUE, RED = (255, 0, 0), (0, 0, 255)       # BGR

    def __init__(self):
        super().__init__("event_recorder")

        # ───────── parameters ────────────────────────────────────────────
        self.declare_parameter("output_dir",      "dvs_recording")
        self.declare_parameter("frame_interval_ms", 10000)   # 10 ms

        out = Path(self.get_parameter("output_dir").value)
        self.frames_dir = out / "event_frames"
        self.rgb_dir    = out / "rgb"
        for d in (out, self.frames_dir, self.rgb_dir):
            d.mkdir(parents=True, exist_ok=True)

        self.dt = self.get_parameter("frame_interval_ms").value / 1000.0

        # ───────── state ────────────────────────────────────────────────
        self.decoder    = ecp.Decoder()
        self.width      = None
        self.height     = None
        self.recording  = False

        # raw buffers
        self.ev_t, self.ev_x, self.ev_y, self.ev_p = [], [], [], []
        self.frame_buf     = []     # (x, y, p) since last flush
        self.frame_count   = 0
        self.rgb_count     = 0

        self.bridge = CvBridge()
        cv2.namedWindow("recorder_ctrl", cv2.WINDOW_NORMAL)
        cv2.imshow("recorder_ctrl",
                   np.zeros((40, 160, 3), np.uint8))
        cv2.displayOverlay("recorder_ctrl",
                           "press 's' to start, 'q' to quit", 0)

        # ───────── ROS I/O ──────────────────────────────────────────────
        self.create_subscription(EventPacket,
                                 "/dvs/events",
                                 self._events_cb, 10)
        self.create_subscription(Image,
                                 "/camera/image_raw",
                                 self._rgb_cb,   10)
        self.create_timer(self.dt, self._flush_frame)
        self.create_timer(0.05, self._check_keys)

        self.get_logger().info("🟢  recorder ready — wait for 's'")

    # ────────────────────────────────────────────────────────────────────
    def _events_cb(self, msg: EventPacket):
        self.decoder.decode(msg)
        cd = self.decoder.get_cd_events()
        if cd.size == 0:
            return

        # geometry (msg may have 0 × 0)
        if self.width in (None, 0) or self.height in (None, 0):
            self.width  = int(cd["x"].max()) + 1
            self.height = int(cd["y"].max()) + 1
        else:
            self.width  = max(self.width,  int(cd["x"].max()) + 1)
            self.height = max(self.height, int(cd["y"].max()) + 1)

        if not self.recording:
            return

        # append raw
        self.ev_t.extend(cd["t"])
        self.ev_x.extend(cd["x"])
        self.ev_y.extend(cd["y"])
        self.ev_p.extend(cd["p"])

        # for frame
        self.frame_buf.extend(zip(cd["x"], cd["y"], cd["p"]))

    # ────────────────────────────────────────────────────────────────────
    def _rgb_cb(self, msg: Image):
        if not self.recording:
            return
        rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        fname = self.rgb_dir / f"rgb_{self.rgb_count:06d}.png"
        cv2.imwrite(str(fname), rgb)
        self.rgb_count += 1

    # ────────────────────────────────────────────────────────────────────
    def _flush_frame(self):
        if not self.recording or not self.frame_buf:
            return
        if self.width in (None, 0) or self.height in (None, 0):
            return
        img = np.zeros((self.height, self.width, 3), np.uint8)
        for x, y, p in self.frame_buf:
            if y < self.height and x < self.width:      # safety
                img[y, x] = self.BLUE if p else self.RED
        fname = self.frames_dir / f"frame_{self.frame_count:06d}.png"
        cv2.imwrite(str(fname), img)
        self.frame_buf.clear()
        self.frame_count += 1

    # ────────────────────────────────────────────────────────────────────
    def _check_keys(self):
        k = cv2.waitKey(1) & 0xFF
        if k == ord('s'):
            self.recording = not self.recording
            state = "ON" if self.recording else "OFF"
            self.get_logger().info(f"🟡 recording {state}")
        elif k == ord('q'):
            self.get_logger().info("🔴 quitting …")
            self._finish()
            rclpy.shutdown()

    # ────────────────────────────────────────────────────────────────────
    def _finish(self):
        if not self.ev_t:
            return
        out = (Path(self.get_parameter("output_dir").value) /
               "events.npz")
        np.savez_compressed(out,
                            t=np.asarray(self.ev_t, dtype=np.int64),
                            x=np.asarray(self.ev_x, dtype=np.uint16),
                            y=np.asarray(self.ev_y, dtype=np.uint16),
                            p=np.asarray(self.ev_p, dtype=np.int8))
        self.get_logger().info(f"💾 wrote {out} "
                               f"({len(self.ev_t)} events)")

def main():
    rclpy.init()
    node = EventRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._finish()
    finally:
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
