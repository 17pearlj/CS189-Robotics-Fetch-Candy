"""
2018-02: Created by Julia Ebert
"""

import cv2
import rospy
import numpy as np

from ar_track_alvar_msgs.msg import AlvarMarkers

# These are the only tag IDs that are being considered here
VALID_IDS = range(18)


class ARTags:
    """
    Identify AR tags, calculate how far away they are, and keep track of the IDs of all the tags seen.
    """

    def __init__(self):

        # Initialize the node
        rospy.init_node('ARTags', anonymous=False)

        # Keep track of all the markers seen
        # This is a dictionary of tag_id: distance pairs
        # If the marker is lost, the value is set to None, but a marker is never removed
        self.markers = {}

        # Tell user how to stop TurtleBot
        rospy.loginfo("To stop TurtleBot CTRL + C")
        # Call this method on rospy shutdown
        rospy.on_shutdown(self.shutdown)

        # Tick at 5 Hz
        self.rate = rospy.Rate(5)

        # Subscribe to topic for AR tags
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.process_ar_tags)

    def run(self):
        """
        Run the robot until Ctrl+C
        We're not moving, so not much happens here
        :return: None
        """
        while not rospy.is_shutdown():
            self.print_markers()
            self.rate.sleep()
            
    def dist(self, pos):
        """
        Get a the distance to a position from the origin. This can be any number of dimensions
        :param pos: Tuple of position
        :return: Float distance of position from origin
        """
        return np.sqrt(sum([i**2 for i in pos]))

    def process_ar_tags(self, data):
        """
        Process the AR tag information.
        :param data: AlvarMarkers message telling you where multiple individual AR tags are
        :return: None
        """
        # Set the position for all the markers that are in the received message
        for marker in data.markers:
            if marker.id in VALID_IDS:
                pos = marker.pose.pose.position
                distance = self.dist((pos.x, pos.y, pos.z))
                self.markers[marker.id] = distance
        # Set the position to None for all the previously seen markers that weren't in the message
        all_seen_ids = self.markers.keys()
        seen_now_ids = [m.id for m in data.markers]
        missing_ids = list(set(all_seen_ids) - set(seen_now_ids))
        self.markers.update(dict.fromkeys(missing_ids, None))
        
    def print_markers(self):
        """
        Print a nicely-formatted presentation of all the tags ever seen
        :return: None
        """
        if len(self.markers) == 0:
            print "\nNo markers seen"
        else:
            print "\nMARKERS:"
            missing_ids = []
            for tag_id, dist in self.markers.iteritems():
                if dist is not None:
                    print "{}:\t{:.2f} m".format(tag_id, dist)
                else:
                    missing_ids.append(tag_id)
            if len(missing_ids) > 0:
                print "Previously seen tags:", ', '.join([str(i) for i in missing_ids])
        print '----------------------------------------------------'

    def shutdown(self):
        """
        Pre-shutdown routine. Stops the robot before rospy.shutdown
        We don't have to pass an empty Twist because we're not doing
        anything with movement
        :return: None
        """
        cv2.destroyAllWindows()
        rospy.loginfo("Stop")
        # wait for robot to stop before shutdown
        rospy.sleep(5)


if __name__ == '__main__':
    try:
        robot = ARTags()
        robot.run()

    except Exception, err:
        rospy.loginfo("ARTags node terminated")
        print err
