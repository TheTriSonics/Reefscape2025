import robotpy_apriltag as ra

apriltag_layout = ra.AprilTagFieldLayout.loadField(
    ra.AprilTagField.k2025ReefscapeWelded
)

w = apriltag_layout.getFieldWidth()
l = apriltag_layout.getFieldLength()
print(w, l)
