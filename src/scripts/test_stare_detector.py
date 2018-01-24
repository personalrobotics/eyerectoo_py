#!/usr/bin/env python
import sys
from csv_data_stream import CSVDataStream
import pdb

if __name__ == "__main__":

    csv_path = sys.argv[1]

    data_stream = CSVDataStream(csv_path)
    stare_detector = StareDetector()

    while (True):
        new_data_vector = data_stream.read()
        # Data stream will return None when reading past end of CSV.
        if (new_data_vector == None):
            break

        stare_marker_id = stare_detector.check_if_staring(new_data_vector.timestamp, new_data_vector.gaze_x, new_data_vector.gaze_y, new_data_vector.aruco_IDs, new_data_vector.aruco_X_vals, new_data_vector.aruco_Y_vals)
        if (stare_marker_id):
            print("STARING AT MARKER " + str(stare_marker_id))
