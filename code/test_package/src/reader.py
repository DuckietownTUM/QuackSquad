import cv2
import numpy as np
import os

image_path = "/Users/andreapellegrin/Desktop/apriltag_2.jpg"
image = cv2.imread(image_path)

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
parameters = cv2.aruco.DetectorParameters()

detector = cv2.aruco.ArucoDetector(dictionary, parameters)
corners, ids, rejected = detector.detectMarkers(image)

if ids is not None:
    cv2.aruco.drawDetectedMarkers(image, corners, ids)

    # Draw tag ID at the center of each detected tag
    for i, corner in enumerate(corners):
        center = np.mean(corner[0], axis=0)  # Calculate the center of the tag
        cv2.putText(image, f"ID: {ids[i][0]}", (int(center[0]), int(center[1])),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    print("Detected AprilTags:")
    for i, tag_id in enumerate(ids):
        print(f" - Tag ID {tag_id[0]} at corners: {corners[i]}")
else:
    print("No AprilTags detected in the image.")

# Prepare the output file name
base, ext = os.path.splitext(image_path)
output_path = f"{base}_marked{ext}"

# Save the image with detected markers
cv2.imwrite(output_path, image)
print(f"Marked image saved as: {output_path}")