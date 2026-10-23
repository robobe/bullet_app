"""Receive Bullet camera images, detect red targets, and publish bounding boxes."""

import textwrap
import argparse
import os
from pathlib import Path
import time

import cv2
import numpy as np
import zmq
from bullet_msgs import load_transport
from .detector import detect_red_box

from .publisher import BboxPublisher
from .receiver import ImageReceiver


def draw_debug(image, bbox, status):
    """Annotate a copy so display overlays never enter the detector."""
    display = image.copy()
    if bbox is not None:
        x, y, width, height = bbox
        cv2.rectangle(display, (x, y), (x + width - 1, y + height - 1), (0, 255, 255), 2)
    lines = textwrap.wrap(status, width=max(20, (display.shape[1] - 16) // 10))
    cv2.rectangle(display, (0, 0), (display.shape[1] - 1, min(display.shape[0] - 1, len(lines) * 22 + 6)),
                  (15, 25, 30), -1)
    for index, line in enumerate(lines):
        cv2.putText(display, line, (8, 23 + index * 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (80, 240, 100), 1, cv2.LINE_AA)
    return display


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--transport-config", type=Path, required=True)
    parser.add_argument("--topic", default="camera/front/image")
    parser.add_argument("--debug", action="store_true", help="Show camera frames and target bounding box")
    parser.add_argument("--bbox-endpoint", default="tcp://127.0.0.1:5558", help="Loopback TCP endpoint to bind for bbox results")
    parser.add_argument("--mission-config", type=Path, help="Enable app-owned takeoff/hover/LOS using this navigator profile")
    args = parser.parse_args(argv)
    receiver = publisher = None
    window_open = False
    window = f"Bullet app: {args.topic} ({os.getpid()})"
    try:
        transport = load_transport(args.transport_config)
        if args.bbox_endpoint in transport.values():
            raise ValueError("bbox endpoint must differ from transport endpoints")
        if args.mission_config is not None:
            from .runtime import run_mission
            return run_mission(args, transport)
        publisher = BboxPublisher(args.bbox_endpoint)
        receiver = ImageReceiver(transport["image_endpoint"], args.topic)
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        bbox = last_frame = None
        last_report = time.monotonic()
        sequence = 0
        if args.debug:
            cv2.namedWindow(window, cv2.WINDOW_NORMAL)
            window_open = True
        print(f'Subscribed to {args.topic} on {transport["image_endpoint"]}, SHM pixels', flush=True)
        print(f"Publishing tracking/front/bbox on {args.bbox_endpoint}", flush=True)
        while True:
            result = receiver.poll()
            now = time.monotonic()
            if result is not None:
                metadata, image = result
                bbox = detect_red_box(image)
                publisher.publish(metadata, bbox, args.topic)
                sequence = metadata["sequence"]
                last_frame = now
            status = "Waiting for frames" if last_frame is None else ("Live" if now - last_frame < 1 else "Stale")
            text = f"{status} | Frame {sequence} | bbox={bbox}"
            if now - last_report >= 1:
                print(text, flush=True)
                last_report = now
            if args.debug:
                cv2.imshow(window, draw_debug(image, bbox, status))
                if cv2.waitKey(10) & 0xff in (ord("q"), 27):
                    break
                if cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                    break
            elif result is None:
                time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    except (OSError, ValueError, zmq.ZMQError, cv2.error) as error:
        print(f"Bullet app failed: {error}", flush=True)
        return 1
    finally:
        if receiver is not None:
            receiver.close()
        if publisher is not None:
            publisher.close()
        if window_open:
            cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
