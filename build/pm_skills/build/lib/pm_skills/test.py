#!/usr/bin/env python3

import argparse
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import Image
import numpy as np

def roi_form_image(image):
    hight_img, wight_img, _ = image.shape
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray_blur = cv2.GaussianBlur(gray, (5, 5), 0)

    # Try multiple binary variants to make contour detection robust.
    binaries = []
    _, bin_inv_fixed = cv2.threshold(gray_blur, 50, 255, cv2.THRESH_BINARY_INV)
    binaries.append(bin_inv_fixed)

    _, bin_otsu_inv = cv2.threshold(
        gray_blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
    )
    binaries.append(bin_otsu_inv)

    _, bin_otsu = cv2.threshold(
        gray_blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
    )
    binaries.append(bin_otsu)

    img_area = float(hight_img * wight_img)
    c = None
    best_area = -1.0
    fallback_c = None
    fallback_area = -1.0
    for binary in binaries:
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            continue
        for candidate in contours:
            area = cv2.contourArea(candidate)
            if area <= 0:
                continue

            if area > fallback_area:
                fallback_area = area
                fallback_c = candidate

            bx, by, bw, bh = cv2.boundingRect(candidate)
            bbox_area = float(bw * bh)
            touches_all_edges = (
                bx <= 0
                and by <= 0
                and (bx + bw) >= wight_img
                and (by + bh) >= hight_img
            )
            almost_full_frame = bbox_area >= (0.98 * img_area)
            if touches_all_edges or almost_full_frame:
                continue

            if area > best_area:
                best_area = area
                c = candidate

    if c is None:
        c = fallback_c

    if c is None:
        #use full image so sharpness can still be computed.
        return [image]

    x, y, w, h = cv2.boundingRect(c)

    top_margin = int(w / 10)
    right_margin = int(h / 10)
    bottom_margin = int(w / 10)
    left_margin = int(h / 10)

    x1 = max(x - left_margin, 0)
    y1 = max(y - top_margin, 0)
    x2 = min(x + w + right_margin, wight_img - 1)
    y2 = min(y + h + bottom_margin, hight_img - 1)

    rois = []

    touches_left = x <= 0
    touches_top = y <= 0
    touches_right = (x + w) >= wight_img
    touches_bottom = (y + h) >= hight_img

    # Add only the side ROIs that are not touching the image border.
    if not touches_left:
        roi2 = image[y1:y2, max(x - right_margin, 0):x + right_margin]
        if roi2.size > 0:
            rois.append(roi2)

    if not touches_bottom:
        roi5 = image[y + h - bottom_margin:y2, x + right_margin:x + w - left_margin]
        if roi5.size > 0:
            rois.append(roi5)

    if not touches_top:
        roi3 = image[y1:y + top_margin, x + right_margin:min(x + w + right_margin, wight_img)]
        if roi3.size > 0:
            rois.append(roi3)

    if not touches_right:
        roi4 = image[y + bottom_margin:y2, x + w - left_margin:x2]
        if roi4.size > 0:
            rois.append(roi4)

    if not rois:
        # Fallback so sharpness can still be computed when no side ROI exists.
        return [image]

    return rois


def laplacian_for_roi(rois):
    sharp_values = []
    for roi in rois:
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        lap = cv2.Laplacian(gray, cv2.CV_64F)
        sharp_values.append(lap.var())
    value = 0.0
    for sharp in sharp_values:
        value += sharp
    value /= len(sharp_values)
    return value
def tenengrad_for_roi(rois):
    sharp_values = []
    for roi in rois:
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
        tenengrad = np.sqrt(sobelx**2 + sobely**2)
        sharp_values.append(np.mean(tenengrad))
    value = 0.0
    for sharp in sharp_values:
        value += sharp
    value /= len(sharp_values)
    return value

def show_rois(rois):
    for i, roi in enumerate(rois):
        cv2.imshow(f"ROI {i}", roi)
    print("Showing ROIs. Press any key in an ROI window to continue.")
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="Compute image sharpness from a ROS topic.")
    parser.add_argument("--topic", default="/Image_Cam1_raw")
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--save", action="store_true", help="Save the received image.")
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Disable ROI visualization windows.",
    )
    parser.add_argument(
        "--output",
        default=str(Path(__file__).resolve().parent / "camframe.png"),
        help="Output PNG path used when --save is set.",
    )
    args = parser.parse_args()

    rclpy.init()
    node = Node("cam1_sharpness_test")
    bridge = CvBridge()

    try:
        success, msg = wait_for_message(
            Image,
            node,
            args.topic,
            time_to_wait=args.timeout,
        )
        if not success or msg is None:
            raise TimeoutError(f"No image received on '{args.topic}' within {args.timeout}s")

        image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if args.save:
            output_path = Path(args.output).expanduser().resolve()
            output_path.parent.mkdir(parents=True, exist_ok=True)
            if not cv2.imwrite(str(output_path), image):
                raise RuntimeError(f"Failed to save image to '{output_path}'")
            print(f"Saved image: {output_path}")

        rois = roi_form_image(image)
        if not args.no_show:
            show_rois(rois)
        sharpness = float(tenengrad_for_roi(rois))
        print(f"Sharpness: {sharpness}")
        return sharpness
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
        raise SystemExit(0)
    except Exception as exc:
        print(f"[FAIL] {exc}")
        raise SystemExit(1)
