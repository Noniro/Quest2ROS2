#!/usr/bin/env python3
"""
lucid_camera_node.py — publish a LUCID (Arena SDK) camera as a ROS2 image topic.

The Lucid Triton (e.g. TRI122S-C, 12.2 MP) is a raw GigE Vision camera: it talks
to the Arena SDK directly, NOT to ROS. This node uses the `arena_api` bindings to
grab frames and republish them as sensor_msgs/Image so the rest of the stack
(episode_recorder, rviz, …) can consume it like any other ROS camera.

Run it via the wrapper `lucid_camera_start.sh`, which puts the x64 Arena SDK on
the linker path and calls `~/lucid_venv/bin/python` on this file.

  NOTE: a GigE camera allows ONE controlling app — close ArenaView before running.

Performance notes (TRI122S-C, learned the hard way):
  * No on-device binning/decimation (Binning maxes at 1). The only size knobs are
    ROI crop (loses FoV) — so we transfer full frames and downscale on the host.
  * `buffer.data` returns a slow Python list (~160 ms/frame). Read `buffer.pdata`
    zero-copy with np.ctypeslib instead (~17 ms/frame).
  * Default mode 'bayer': stream BayerRG8 (1/3 the bytes of RGB8) and collapse each
    2x2 RGGB block to one RGB pixel in numpy (cheap, half-res), then resize. Use
    --mode rgb for on-device RGB8 (no host debayer) if you prefer.
  * Full-res frame rate is limited by EXPOSURE in dim scenes. Lower --exposure-us
    (and raise gain / add light) to get closer to the sensor's ~9 fps.
"""
import argparse

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import cv2
from arena_api.system import system


def _try_set(nodemap, name, value, log):
    """Set a GenICam node if present/writable; log the outcome, never raise."""
    try:
        node = nodemap.get_node(name)
        if node is None:
            log.warn(f'[lucid] node {name!r} absent — skipped')
            return False
        node.value = value
        log.info(f'[lucid] {name} = {value}')
        return True
    except Exception as e:  # noqa: BLE001 — GenICam raises many types
        log.warn(f'[lucid] could not set {name}={value}: {str(e).strip()[:60]}')
        return False


class LucidCamera(Node):
    def __init__(self, args):
        super().__init__('lucid_camera')
        self.args = args
        self.pub = self.create_publisher(Image, args.topic, 10)
        log = self.get_logger()

        devices = system.create_device()
        if not devices:
            raise RuntimeError('no LUCID device — powered/connected? ArenaView closed?')
        self.device = devices[0]
        nm = self.device.nodemap
        log.info(f"[lucid] {nm['DeviceModelName'].value} SN {nm['DeviceSerialNumber'].value}")

        # pixel format: 'bayer' mode streams BayerRG8 (light) + host debayer;
        # 'rgb' streams RGB8 on-device; 'mono' streams Mono8.
        self.mode = args.mode
        fmt = {'bayer': 'BayerRG8', 'rgb': 'RGB8', 'mono': 'Mono8'}[self.mode]
        _try_set(nm, 'PixelFormat', fmt, log)

        # exposure / gain — short fixed exposure keeps frame rate up in dim scenes
        if args.exposure_us > 0:
            _try_set(nm, 'ExposureAuto', 'Off', log)
            _try_set(nm, 'ExposureTime', float(args.exposure_us), log)
        else:
            _try_set(nm, 'ExposureAuto', 'Continuous', log)
        _try_set(nm, 'GainAuto', 'Continuous', log)

        self.w = nm['Width'].value - nm['Width'].value % 2
        self.h = nm['Height'].value - nm['Height'].value % 2
        self.chans = 1 if self.mode == 'mono' else 3
        self.out_encoding = {'bayer': 'rgb8', 'rgb': 'rgb8', 'mono': 'mono8'}[self.mode]
        log.info(f'[lucid] streaming {self.w}x{self.h} {fmt} (mode={self.mode})')

        # host downscale target width (keeps aspect); 0 = native
        self.resize_to = None
        src_w = self.w // 2 if self.mode == 'bayer' else self.w
        src_h = self.h // 2 if self.mode == 'bayer' else self.h
        if args.width > 0 and src_w > args.width:
            self.resize_to = (args.width, int(round(src_h * args.width / src_w)))
            log.info(f'[lucid] host resize -> {self.resize_to[0]}x{self.resize_to[1]}')

        # GigE stream hygiene
        sm = self.device.tl_stream_nodemap
        _try_set(sm, 'StreamBufferHandlingMode', 'NewestOnly', log)
        _try_set(sm, 'StreamAutoNegotiatePacketSize', True, log)
        _try_set(sm, 'StreamPacketResendEnable', True, log)

        self.device.start_stream(args.buffers)
        self.frames = 0
        self.timer = self.create_timer(1.0 / max(args.fps, 1), self._grab)
        log.info(f'[lucid] publishing on {args.topic}')

    def _grab(self):
        if not rclpy.ok():
            return
        try:
            buf = self.device.get_buffer(timeout=300)
        except Exception:                        # no buffer ready this tick
            return
        try:
            n = self.h * self.w * (1 if self.mode != 'rgb' else 3)
            flat = np.ctypeslib.as_array(buf.pdata, shape=(n,))
            if self.mode == 'rgb':
                raw = flat.reshape(self.h, self.w, 3).copy()
            else:
                raw = flat.reshape(self.h, self.w).copy()
        finally:
            self.device.requeue_buffer(buf)

        img = self._process(raw)
        self._publish(img)
        self.frames += 1
        if self.frames % 30 == 0:
            self.get_logger().info(f'[lucid] …{self.frames} frames')

    def _process(self, raw):
        if self.mode == 'bayer':
            # BayerRG8 RGGB -> half-res RGB, numpy only (no full debayer)
            r = raw[0::2, 0::2]
            g = ((raw[0::2, 1::2].astype(np.uint16) + raw[1::2, 0::2]) >> 1).astype(np.uint8)
            b = raw[1::2, 1::2]
            img = np.dstack([r, g, b])
        else:
            img = raw
        if self.resize_to is not None:
            img = cv2.resize(img, self.resize_to, interpolation=cv2.INTER_AREA)
        return img

    def _publish(self, img):
        m = Image()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.args.frame_id
        m.height, m.width = int(img.shape[0]), int(img.shape[1])
        m.encoding = self.out_encoding
        m.step = m.width * (1 if self.out_encoding == 'mono8' else 3)
        m.is_bigendian = 0
        m.data = np.ascontiguousarray(img).tobytes()
        self.pub.publish(m)

    def shutdown(self):
        try:
            self.device.stop_stream()
        except Exception:
            pass
        try:
            system.destroy_device()
        except Exception:
            pass


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--topic', default='/lucid/image_raw')
    p.add_argument('--frame-id', default='lucid_optical_frame')
    p.add_argument('--mode', default='bayer', choices=['bayer', 'rgb', 'mono'],
                   help='bayer=BayerRG8+host debayer (default, lightest), rgb=RGB8, mono=Mono8')
    p.add_argument('--width', type=int, default=640,
                   help='host downscale target width, keeps aspect (0=native)')
    p.add_argument('--fps', type=int, default=15, help='publish-attempt rate (Hz)')
    p.add_argument('--exposure-us', type=float, default=0.0,
                   help='fixed exposure in microseconds (0=auto). Lower = higher fps in dim scenes')
    p.add_argument('--buffers', type=int, default=4)
    args = p.parse_args()

    rclpy.init()
    node = LucidCamera(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
