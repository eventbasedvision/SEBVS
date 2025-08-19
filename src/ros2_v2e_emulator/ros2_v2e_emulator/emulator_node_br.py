#!/usr/bin/env python3
"""
ROS2 node wrapping SensorsINI v2e, full resolution, no event trails:
 • APS & DVS side by side in one window
 • Blue = ON polarity, Red = OFF polarity
 • Per-frame DVS canvas is freshly cleared (no trace)
 • Optional blur & denoise to suppress noise
 • GPU-first, CPU fallback on OOM
 • Per-frame latency + event count overlay
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

# Import path for your v2e build
try:
    from v2ecore.emulator import EventEmulator
except ImportError:
    from v2ecore.event_emulator import EventEmulator

FRAME_FUNCS = ("new_frame", "next_frame", "generate_events", "run")

class SafeEmu:
    def __init__(self, **params):
        self.cpu = EventEmulator(**params)
        self.gpu = EventEmulator(**params) if torch.cuda.is_available() else None
        self.on_gpu = self.gpu is not None

        sim = self.gpu or self.cpu
        for name in FRAME_FUNCS:
            if callable(getattr(sim, name, None)):
                self.fn_name = name
                break
        else:
            raise RuntimeError("No per-frame API found")
        
    def events(self, img, ts=None):
        sim = self.gpu if self.on_gpu else self.cpu
        fn  = getattr(sim, self.fn_name)

        try:
            return fn(img, ts) if ts is not None else fn(img)
        except TypeError:  # in case ts is unsupported
            return fn(img)
        except torch.cuda.OutOfMemoryError:
            torch.cuda.empty_cache()
            self.on_gpu = False
            print("CUDA OOM → falling back to CPU")
            return self.events(img, ts)
        

class V2ENode(Node):
    BLUE = (255,0,0)  # ON
    RED  = (0,0,255)  # OFF

    def __init__(self):
        super().__init__("emulator_node_br")
        self.bridge = CvBridge()

        # — Parameters —
        self.declare_parameter("pos_thres",    0.2)
        self.declare_parameter("neg_thres",    0.2)
        self.declare_parameter("sigma_thres",  0.03)
        self.declare_parameter("cutoff_hz",    25.0)
        self.declare_parameter("leak_rate_hz", 0.0)
        self.declare_parameter("blur",         False)
        self.declare_parameter("denoise",      False)

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

        self.sub = self.create_subscription(
            Image, "/camera/image_raw", self.cb, 10)
        self.pub = self.create_publisher(
            EventPacket, "/dvs/events", 10)

        # One combined display window
        cv2.namedWindow("APS | DVS", cv2.WINDOW_NORMAL)

        # State
        self.prev_ros = None
        self.rel_t    = 0.0
        self.lat_ms   = 0.0
        self.evt_cnt  = 0

    def cb(self, msg: Image):
        # 1) APS: raw full-res camera frame
        gray = self.bridge.imgmsg_to_cv2(msg, "mono8")
        aps_bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        # 2) Relative timestamp (clamped)
        ros_t = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        if self.prev_ros is None:
            self.prev_ros = ros_t
            # show APS alone first
            empty = np.zeros_like(aps_bgr)
            cv2.imshow("APS | DVS", np.hstack((aps_bgr, empty)))
            cv2.waitKey(1)
            return
        dt = max(1e-4, min(ros_t - self.prev_ros, 0.1))
        self.prev_ros = ros_t
        self.rel_t   += dt

        # 3) Optional blur
        inp = gray.astype(np.float32)
        if self.get_parameter("blur").value:
            inp = cv2.GaussianBlur(inp, (3,3), 0)

        # 4) Generate events + measure time
        t0 = time.perf_counter()
        ev = self.emu.events(inp, self.rel_t)
        self.lat_ms = (time.perf_counter() - t0)*1000.0

        # 5) If no events, show blank DVS half
        if ev is None or len(ev) == 0:
            empty = np.zeros_like(aps_bgr)
            cv2.imshow("APS | DVS", np.hstack((aps_bgr, empty)))
            cv2.waitKey(1)
            return
        if isinstance(ev, np.ndarray):
            ev = ev.tolist()
        self.evt_cnt = len(ev)

        # 6) Build ON/OFF masks correctly (+1/–1)
        arr = np.array(ev, dtype=np.float32)
        xs, ys, ps = arr[:,1].astype(int), arr[:,2].astype(int), arr[:,3]
        h, w = gray.shape
        on_mask, off_mask = np.zeros((h,w),np.uint8), np.zeros((h,w),np.uint8)
        on_mask [ys[ps> 0], xs[ps> 0]] = 255
        off_mask[ys[ps< 0], xs[ps< 0]] = 255

        if self.get_parameter("denoise").value:
            k = np.ones((3,3), np.uint8)
            on_mask  = cv2.morphologyEx(on_mask,  cv2.MORPH_OPEN, k)
            off_mask = cv2.morphologyEx(off_mask, cv2.MORPH_OPEN, k)

        # 7) Draw fresh DVS canvas
        dvs = np.zeros((h,w,3), np.uint8)
        dvs[on_mask>0]  = self.BLUE
        dvs[off_mask>0] = self.RED

        # 8) Pack & publish EventPacket
        pkt = EventPacket()
        pkt.header    = msg.header
        pkt.time_base = int(self.rel_t*1e9)
        pkt.encoding  = "mono"
        buf = bytearray()
        for tf, xf, yf, pf in ev:
            x = int(xf); y = int(yf)
            p = 1 if pf>0 else 0
            dt_us = int(tf*1e6)&0xFFFFFFFF
            w64   = (p<<63)|(y<<48)|(x<<32)|dt_us
            buf.extend(w64.to_bytes(8,"little"))
        pkt.events = list(buf)
        self.pub.publish(pkt)

        # 9) Overlay latency & count
        txt = f"{self.lat_ms:.1f} ms   {self.evt_cnt} evts"
        cv2.putText(dvs, txt, (10,20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (255,255,255), 2, cv2.LINE_AA)

        # 10) Show APS | DVS
        combined = np.hstack((aps_bgr, dvs))
        cv2.imshow("APS | DVS", combined)
        cv2.waitKey(1)

def main():
    rclpy.init()
    rclpy.spin(V2ENode())
    rclpy.shutdown()

if __name__=="__main__":
    main()
