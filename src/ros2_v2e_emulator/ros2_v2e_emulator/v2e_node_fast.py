#!/usr/bin/env python3
"""
ROS 2 node wrapping the SensorsINI v2e emulator, with:
 • full parameter docs & defaults
 • downsampling & blur toggles for speed
 • GPU-first & CPU-fallback logic
 • black background, blue = ON, red = OFF
 • per-frame latency + event-count overlay
 • NEW: dot_radius (thicker events) and viz_scale (window zoom)
"""
import time
import cv2
import numpy as np
import torch
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from event_camera_msgs.msg import EventPacket
from cv_bridge import CvBridge
# Import EventEmulator from whichever path your version uses
try:
    from v2ecore.emulator import EventEmulator        # newer layouts
except ImportError:
    from v2ecore.event_emulator import EventEmulator  # older layouts
FRAME_FUNCS = ("new_frame", "next_frame", "generate_events", "run")
def try_gpu(params):
    """Return a GPU-backed EventEmulator if possible, else None."""
    if not torch.cuda.is_available():
        return None
    try:
        return EventEmulator(**params)  # old builds ignore extra kwargs
    except Exception as e:
        print("GPU init failed → CPU only:", e)
        return None
class SafeEmu:
    """Wrap CPU/GPU emulator and auto-fallback on OOM."""
    def __init__(self, **params):
        self.cpu = EventEmulator(**params)
        self.gpu = try_gpu(params)
        self.on_gpu = self.gpu is not None
        sim = self.gpu or self.cpu
        # pick first callable frame function
        for name in FRAME_FUNCS:
            fn = getattr(sim, name, None)
            if callable(fn):
                self.fn_name = name
                self.fn = fn
                break
        else:
            raise RuntimeError("No per-frame API found on EventEmulator")
    def events(self, img, ts):
        sim = self.gpu if self.on_gpu else self.cpu
        fn = getattr(sim, self.fn_name)
        try:
            return fn(img, ts)
        except TypeError:
            return fn(img)
        except torch.cuda.OutOfMemoryError:
            torch.cuda.empty_cache()
            self.on_gpu = False
            print("CUDA OOM → using CPU this frame")
            return self.events(img, ts)
class V2ENode(Node):
    BLUE = (255, 0, 0)    # ON events
    RED  = (0, 0, 255)    # OFF events
    def __init__(self):
        super().__init__("v2e_node")
        self.bridge = CvBridge()
        # ---- Declare parameters with sensible defaults ----
        self.declare_parameter("pos_thres",    0.9)
        self.declare_parameter("neg_thres",    0.9)
        self.declare_parameter("sigma_thres",  0.05)
        self.declare_parameter("cutoff_hz",    50.0)
        self.declare_parameter("leak_rate_hz", 0.0)    # disable leak
        self.declare_parameter("downsample",   1.0)
        self.declare_parameter("blur",         False)
        # NEW visualisation knobs
        self.declare_parameter("dot_radius", 0)   # 0 → 1-pixel dot
        self.declare_parameter("viz_scale",  1)   # 1 → native size
        p = self.get_parameter
        emu_args = dict(
            pos_thres    = p("pos_thres").value,
            neg_thres    = p("neg_thres").value,
            sigma_thres  = p("sigma_thres").value,
            cutoff_hz    = p("cutoff_hz").value,
            leak_rate_hz = p("leak_rate_hz").value,
        )
        self.emu = SafeEmu(**emu_args)
        self.get_logger().info(
            f"Using {self.emu.fn_name} on "
            f"{'GPU' if self.emu.on_gpu else 'CPU'}"
        )
        # ROS 2 I/O
        self.sub = self.create_subscription(
            Image, "/ee_camera/image_raw", self.cb, 10)
        self.pub = self.create_publisher(
            EventPacket, "/dvs/events", 10)
        # OpenCV windows
        cv2.namedWindow("APS", cv2.WINDOW_NORMAL)
        cv2.namedWindow("DVS", cv2.WINDOW_NORMAL)
        # timing state
        self.prev_ros = None
        self.rel_t    = 0.0
        self.lat_ms   = 0.0
        self.evt_cnt  = 0
    # ---------------------------------------------------------------------- #
    #                                Callback                                #
    # ---------------------------------------------------------------------- #
    def cb(self, msg: Image):
        # 1) Decode & optional downsample
        gray = self.bridge.imgmsg_to_cv2(msg, "mono8")
        ds = float(self.get_parameter("downsample").value)
        if ds != 1.0:
            gray = cv2.resize(gray, None, fx=ds, fy=ds,
                              interpolation=cv2.INTER_AREA)
        # show APS (camera feed)
        cv2.imshow("APS", cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR))
        cv2.waitKey(1)
        # 2) compute small relative dt (clamped)
        ros_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.prev_ros is None:
            self.prev_ros = ros_t
            return
        dt = max(1e-4, min(ros_t - self.prev_ros, 0.1))
        self.prev_ros = ros_t
        self.rel_t += dt
        # 3) optional blur → float32 input
        inp = gray.astype(np.float32)
        if self.get_parameter("blur").value:
            inp = cv2.GaussianBlur(inp, (3, 3), 0)
        # 4) run v2e + measure latency
        t0 = time.perf_counter()
        ev = self.emu.events(inp, self.rel_t)
        self.lat_ms = (time.perf_counter() - t0) * 1000.0
        # 5) skip if empty or None
        if ev is None or len(ev) == 0:
            return
        if isinstance(ev, np.ndarray):
            ev = ev.tolist()
        self.evt_cnt = len(ev)
        # 6) pack & draw onto BLACK canvas
        h, w = gray.shape
        canvas = np.zeros((h, w, 3), np.uint8)
        pkt = EventPacket()
        pkt.header = msg.header
        pkt.time_base = int(self.rel_t * 1e9)
        pkt.encoding = "mono"
        buf = bytearray()
        R = int(self.get_parameter("dot_radius").value)
        for tf, xf, yf, pf in ev:
            x = int(xf)
            y = int(yf)
            p = int(pf) & 1
            dtu = int(tf * 1e6) & 0xFFFFFFFF
            w64 = (p << 63) | (y << 48) | (x << 32) | dtu
            buf.extend(w64.to_bytes(8, "little"))
            # draw event
            color = self.BLUE if p else self.RED
            if R == 0:
                canvas[y, x] = color
            else:
                cv2.circle(canvas, (x, y), R, color,
                           thickness=-1, lineType=cv2.LINE_8)
        pkt.events = list(buf)
        self.pub.publish(pkt)
        # 7) overlay latency + count & display
        txt = f"{self.lat_ms:.1f} ms   {self.evt_cnt} evts"
        cv2.putText(canvas, txt, (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (255, 255, 255), 2, cv2.LINE_AA)
        s = int(self.get_parameter("viz_scale").value)
        show = canvas if s == 1 else cv2.resize(
            canvas, None, fx=s, fy=s,
            interpolation=cv2.INTER_NEAREST)   # keep blocky pixels crisp
        cv2.imshow("DVS", show)
        cv2.waitKey(1)
# ---------------------------------------------------------------------- #
#                                   main                                 #
# ---------------------------------------------------------------------- #
def main():
    rclpy.init()
    rclpy.spin(V2ENode())
    rclpy.shutdown()
if __name__ == "__main__":
    main()
