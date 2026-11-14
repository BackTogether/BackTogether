#!/usr/bin/env python3
"""Print "ok" while the expected person is in front of the head camera."""

import rospy
from face_rec.msg import face_results

# Must match the folder name of the person's photos in $FACE_DIRECTORY.
EXPECTED_PERSON = "jorge"
LOG_PERIOD_S = 1.0  # /face_results arrives once per camera frame


def on_faces(results: face_results) -> None:
    names = [face.name.lower() for face in results.face_data]
    if EXPECTED_PERSON in names:
        rospy.loginfo_throttle(LOG_PERIOD_S, "ok")


if __name__ == "__main__":
    rospy.init_node("face_check")
    rospy.Subscriber("/face_results", face_results, on_faces, queue_size=1)
    rospy.spin()
