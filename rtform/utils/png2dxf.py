#!/usr/bin/env python3

import cv2
import numpy as np
import ezdxf
import os

def read_and_display_image(png_path):
    # Load the image in grayscale
    img = cv2.imread(png_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"Could not read image at {png_path}")

    # Show the image
    cv2.imshow("Input Image", img)
    print("Press any key on the image window to continue...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return img

def find_contours(img):
    # Convert to binary
    _, binary = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY_INV)
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours

def contours_to_dxf(contours, dxf_path):
    doc = ezdxf.new()
    msp = doc.modelspace()

    for contour in contours:
        points = contour[:, 0, :]  # reshape from (N, 1, 2) to (N, 2)
        if len(points) >= 2:
            # Create a polyline
            polyline = msp.add_lwpolyline(points, close=True)

    doc.saveas(dxf_path)
    print(f"Saved DXF to {dxf_path}")

if __name__ == "__main__":
    png_path = "../assets/cutting_patterns/custom_cutting_pattern_2.png"  # <-- Replace with your actual image file
    dxf_path = os.path.splitext(png_path)[0] + ".dxf"

    image = read_and_display_image(png_path)
    contours = find_contours(image)
    contours_to_dxf(contours, dxf_path)
