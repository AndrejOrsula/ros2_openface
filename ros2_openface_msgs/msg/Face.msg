# Message that contains information obtained from OpenFace
# Please see https://github.com/TadasBaltrusaitis/OpenFace/wiki/Output-Format for more information about the landmarks

# All 2D (image) landmarks of the face, as pixels
Pixel[] landmarks
# Visible 2D (image) landmarks of the face, as pixels
Pixel[] landmarks_visible
# All 3D landmarks of the face, in metres
geometry_msgs/Point[] landmarks_3d

# 3D pose of the head, in metres
geometry_msgs/Pose head_pose

# All 2D (image) landmarks of the eyes, as pixels
Pixel[] eye_landmarks
# Visible 2D (image) landmarks of the eyes, as pixels
Pixel[] eye_landmarks_visible
# All 3D landmarks of the eyes, in metres
geometry_msgs/Point[] eye_landmarks_3d

# Gaze of the left eye, with pupil centre position in metres
geometry_msgs/Pose gaze_left
# Gaze of the right eye, with pupil centre position in metres
geometry_msgs/Pose gaze_right
# Compound gaze of both eyes, taken as the average of between the left and right gaze
geometry_msgs/Vector3 gaze_compound

# List of the recognised Action Unit
ActionUnit[] action_units
