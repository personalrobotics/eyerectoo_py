#!/usr/bin/env python
from msgpack import loads
import json
import rospy
from eyerectoo_stream.msg import Journal
from eyerectoo_capture import EyeRecTooCapture

def parse_journal_data(journal_data):
    ''' Parse EyeRecToo Journal into a Journal message.'''  
    outmsg = Journal()

    # TODO: Should remember in an elegant way which fields are at
    # which index, not have magic numbers like this.

    # Split off the "J"
    outmsg.sync_timestamp =  int(journal_data[0][1:])

    # TODO: Rest of journal fields once we have sync_timestap working.

    return outmsg



if __name__ == "__main__":

    # Create publisher for Journal data from EyeRecToo
    pub_journal = rospy.Publisher('/eyerectoo_journal', Journal, queue_size=10)

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