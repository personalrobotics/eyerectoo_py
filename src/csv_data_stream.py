import pandas
import pdb
from gaze_data_vector import GazeDataVector

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

        # TODO: Aruco stuff. Haha.

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
