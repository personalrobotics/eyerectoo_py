# TODO: This entire script needs to be redone since we handle data processing
# in the Python layer now. 

#!/usr/bin/env python
from msgpack import loads
import json
import rospy
from eyerectoo_stream.msg import Journal
from eyerectoo_capture import EyeRecTooCapture

import pdb

def parse_journal_data(journal_data):
    ''' Parse EyeRecToo Journal into a Journal message.'''
    outmsg = Journal()

    # TODO: Should remember in an elegant way which fields are at
    # which index, not have magic numbers like this.

    # Split off the "J"
    outmsg.sync_timestamp =  int(journal_data[0][1:])

    outmsg.gaze_x = float(journal_data[2])
    outmsg.gaze_y = float(journal_data[3])
    outmsg.valid_gaze = int(journal_data[5]) == 1

    outmsg.field_width = int(journal_data[11])
    outmsg.field_height = int(journal_data[12])

    aruco_markers_present = len(journal_data) > 30
    outmsg.aruco_markers_present = aruco_markers_present
    if aruco_markers_present:
        outmsg.aruco_markers = journal_data[13]
        # Remove the aruco marker data to make indicies consistent
        journal_data = journal_data[:13] + journal_data[14:]
        #pdb.set_trace()


    outmsg.left_pupil_x = float(journal_data[15])
    outmsg.left_pupil_y = float(journal_data[16])
    outmsg.left_pupil_width = float(journal_data[17])
    outmsg.left_pupil_height = float(journal_data[18])
    outmsg.left_pupil_angle = float(journal_data[19])
    outmsg.left_pupil_valid = int(journal_data[20]) == 1

    outmsg.right_pupil_x = float(journal_data[23])
    outmsg.right_pupil_y = float(journal_data[24])
    outmsg.right_pupil_width = float(journal_data[25])
    outmsg.right_pupil_height = float(journal_data[26])
    outmsg.right_pupil_angle = float(journal_data[27])
    outmsg.right_pupil_valid = int(journal_data[28]) == 1

    #pdb.set_trace()

    return outmsg



if __name__ == "__main__":

    # Create publisher for Journal data from EyeRecToo
    pub_journal = rospy.Publisher('eyerectoo_journal', Journal, queue_size=10)

    rospy.loginfo("Starting EyeRecToo journal publisher.")
    print "Starting EyeRecToo journal publisher."

    rospy.init_node('eyerectoo_publish')
    eyerectoo_listener = EyeRecTooCapture()

    while not rospy.is_shutdown():
        journal_data = eyerectoo_listener.read()

        rospy.logdebug("Reading EyeRecToo journal")
        # Parse and publish
        outmsg = parse_journal_data(journal_data)
        pub_journal.publish(outmsg)
