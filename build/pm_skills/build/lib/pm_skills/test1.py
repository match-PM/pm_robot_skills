# #!/usr/bin/env python3

# import argparse
# from pathlib import Path

# import cv2


# def roi_form_image(image):
#     hight_img, wight_img, _ = image.shape
#     print(hight_img, wight_img)
#     gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
#     # _, binary = cv2.threshold(gray, 50, 255, cv2.THRESH_BIARY_INV)

#     contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#     if not contours:
#         # Fallback so sharpness can still be computed.
#         return [image]
#     c = max(contours, key=cv2.contourArea)

#     x, y, w, h = cv2.boundingRect(c)

#     top_margin = int(w / 10)
#     right_margin = int(h / 10)
#     bottom_margin = int(w / 10)
#     left_margin = int(h / 10)

#     x1 = max(x - left_margin, 0)
#     y1 = max(y - top_margin, 0)
#     x2 = min(x + w + right_margin, image.shape[1] - 1)
#     y2 = min(y + h + bottom_margin, image.shape[0] - 1)

#     roi1 = image[y1:y2, x1:x2]

#     rois = [roi1]
#     if not ((x == 0) and (y != 0)):
#         roi2 = image[y1:y2, max(x - right_margin, 0):x + right_margin]
#         roi5 = image[y + h - bottom_margin:y2, x + right_margin:x + w - left_margin]

#         if roi2.size > 0:
#             rois.append(roi2)
#         if roi5.size > 0:
#             rois.append(roi5)

#     if not (x + w == wight_img - 1) and not (y + h == hight_img - 1):
#         roi4 = image[y + bottom_margin:y2, x + w - left_margin:x2]
#         roi3 = image[y1:y + top_margin, x + right_margin:min(x + w + right_margin, wight_img)]
#         if roi3.size > 0:
#             rois.append(roi3)
#         if roi4.size > 0:
#             rois.append(roi4)

#     return rois


# def laplacian_for_roi(rois):
#     sharp_values = []
#     for roi in rois:
#         gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
#         lap = cv2.Laplacian(gray, cv2.CV_64F)
#         sharp_values.append(lap.var())
#     return min(sharp_values)


# def main():
#     default_image = Path(__file__).resolve().parent / "cam1_frame.png"
#     parser = argparse.ArgumentParser(description="Compute sharpness from an image file.")
#     parser.add_argument("--image", default=str(default_image))
#     parser.add_argument("--show", action="store_true")
#     args = parser.parse_args()

#     image_path = Path(args.image).expanduser().resolve()
#     img = cv2.imread(str(image_path))
#     if img is None:
#         raise RuntimeError(f"Failed to load image: {image_path}")

#     rois = roi_form_image(img)
#     sharpness = laplacian_for_roi(rois)
#     print("Sharpness:", sharpness)

#     if args.show:
#         for i, roi in enumerate(rois):
#             cv2.imshow(f"ROI {i}", roi)
#             cv2.waitKey(0)
#         cv2.destroyAllWindows()


# if __name__ == "__main__":
#     main()
