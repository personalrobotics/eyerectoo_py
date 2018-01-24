import pandas
from gaze_data_vector import GazeDataVector

import pdb

# Reads data from a CSV to "mimic" the data stream when getting live gaze data
# from EyeRecToo.
class CSVDataStream():

    def __init__(self, csv_filename):
        # setting UDP socket.
        self.csv_data = pandas.read_csv(csv_filename)
        self.row_index = 0
        self.num_rows = len(self.csv_data["timestamp"])

    def read(self):
        if (self.row_index == self.num_rows):
            # No more CSV to read.
            return None

        new_data_vector = GazeDataVector()
        new_data_vector.timestamp = self.csv_data["timestamp"][self.row_index]
        new_data_vector.gaze_x = self.csv_data["gaze_x"][self.row_index]
        new_data_vector.gaze_y = self.csv_data["gaze_y"][self.row_index]

        new_data_vector.field_width = self.csv_data["field_width"][self.row_index]
        new_data_vector.field_height = self.csv_data["field_height"][self.row_index]

        new_data_vector.aruco_markers_present = self.csv_data["aruco_markers_present"][self.row_index]
        new_data_vector.aruco_IDs = []
        new_data_vector.aruco_X_vals = []
        new_data_vector.aruco_Y_vals = []

        if new_data_vector.aruco_markers_present:
            marker_id_string = self.csv_data["aruco_IDs"][self.row_index]
            for string_id in marker_id_string.split(":"):
                new_data_vector.aruco_IDs.append(int(string_id))

            marker_x_coord_string = self.csv_data["aruco_X_vals"][self.row_index]
            for string_x_coord in marker_x_coord_string,split(":")
                new_data_vector.aruco_X_vals.append(float(string_x_coord))

            marker_y_coord_string = self.csv_data["aruco_Y_vals"][self.row_index]
            for string_y_coord in marker_y_coord_string.split(":"):
                new_data_vector.aruco_Y_vals.append(float(string_y_coord))

        new_data_vector.left_pupil_x = self.csv_data["left_pupil_x"][self.row_index]
        new_data_vector.left_pupil_y = self.csv_data["left_pupil_y"][self.row_index]
        new_data_vector.left_pupil_width = self.csv_data["left_pupil_width"][self.row_index]
        new_data_vector.left_pupil_height = self.csv_data["left_pupil_height"][self.row_index]
        new_data_vector.left_pupil_angle = self.csv_data["left_pupil_angle"][self.row_index]

        new_data_vector.right_pupil_x = self.csv_data["right_pupil_x"][self.row_index]
        new_data_vector.right_pupil_y = self.csv_data["right_pupil_y"][self.row_index]
        new_data_vector.right_pupil_width = self.csv_data["right_pupil_width"][self.row_index]
        new_data_vector.right_pupil_height = self.csv_data["right_pupil_height"][self.row_index]
        new_data_vector.right_pupil_angle = self.csv_data["right_pupil_angle"][self.row_index]

        # Read the next row next time.
        self.row_index += 1

        return new_data_vector
