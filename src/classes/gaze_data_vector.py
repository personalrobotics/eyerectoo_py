import pdb

# Used to wrap a raw UDP packet from EyeRecToo and turns it into an object that
# is *way* more human readable. Can also be set up as an empty struct and filled
# in if UDP data is not given.
class GazeDataVector():

    def __init__(self, udp_string=None):
        if udp_string:
            wrap_udp(udp_string)

    def wrap_udp(udp_string):
        udp_fields = udp_string.split()

        # Split off the "J"
        self.timestamp = int(udp_fields[0][1:])
        self.gaze_x = float(udp_fields[2])
        self.gaze_y = float(udp_fields[3])

        self.field_width = int(udp_fields[11])
        self.field_height = int(udp_fields[12])

        # Handle Aruco Data
        self.aruco_markers_present = len(udp_fields) > 30

        self.aruco_IDs = []
        self.aruco_X_vals = []
        self.aruco_Y_vals = []
        if self.aruco_markers_present:
            aruco_string = udp_fields[13]
            # There's an extra semicolon at the end for some reason.
            aruco_parts = aruco_string.split(';')[:-1]
            for tag_data in aruco_parts:
                tag_ID, tag_coords_str = tag_data.split(':')
                tag_x, tag_y, _ = tag_coords_str.split('x')
                self.aruco_IDs.append(tag_ID)
                self.aruco_X_vals.append(tag_x)
                self.aruco_Y_vals.append(tag_y)

            # Remove the aruco marker data to make next indicies consistent
            udp_fields = udp_fields[:13] + udp_fields[14:]

        self.left_pupil_x = float(udp_fields[15])
        self.left_pupil_y = float(udp_fields[16])
        self.left_pupil_width = float(udp_fields[17])
        self.left_pupil_height = float(udp_fields[18])
        self.left_pupil_angle = float(udp_fields[19])

        self.right_pupil_x = float(udp_fields[23])
        self.right_pupil_y = float(udp_fields[24])
        self.right_pupil_width = float(udp_fields[25])
        self.right_pupil_height = float(udp_fields[26])
        self.right_pupil_angle = float(udp_fields[27])
