#!/usr/bin/env python
import csv
import time
import sys

from live_data_stream import LiveDataStream

import pdb

def transform_data_vector(data_vector):
    ''' Add a line to the CSV from new data.'''
    new_csv_row = []

    new_csv_row.append(str(data_vector.timestamp))
    new_csv_row.append(str(data_vector.gaze_x))
    new_csv_row.append(str(data_vector.gaze_y))

    new_csv_row.append(str(data_vector.field_width))
    new_csv_row.append(str(data_vector.field_height))

    # Handle Aruco Data
    new_csv_row.append(str(data_vector.aruco_markers_present))
    if data_vector.aruco_markers_present:
        # Turn each array into a colon delimited list.
        aruco_IDs = ":".join(data_vector.aruco_IDs)
        aruco_X_vals = ":".join(data_vector.aruco_X_vals)
        aruco_Y_vals = ":".join(data_vector.aruco_Y_vals)

    else:
        aruco_IDs = "None"
        aruco_X_vals = "None"
        aruco_Y_vals = "None"

    new_csv_row.append(aruco_IDs)
    new_csv_row.append(aruco_X_vals)
    new_csv_row.append(aruco_Y_vals)

    new_csv_row.append(str(data_vector.left_pupil_x))
    new_csv_row.append(str(data_vector.left_pupil_y))
    new_csv_row.append(str(data_vector.left_pupil_width))
    new_csv_row.append(str(data_vector.left_pupil_height))
    new_csv_row.append(str(data_vector.left_pupil_angle))

    new_csv_row.append(str(data_vector.right_pupil_x))
    new_csv_row.append(str(data_vector.right_pupil_y))
    new_csv_row.append(str(data_vector.right_pupil_width))
    new_csv_row.append(str(data_vector.right_pupil_height))
    new_csv_row.append(str(data_vector.right_pupil_angle))

    return new_csv_row


if __name__ == "__main__":

    output_path = sys.argv[1]
    # Number of seconds to record for.
    recording_time = int(sys.argv[2])

    title_row = []
    title_row.append("timestamp")
    title_row.append("gaze_x")
    title_row.append("gaze_y")

    title_row.append("field_width")
    title_row.append("field_height")

    title_row.append("aruco_markers_present")
    title_row.append("aruco_IDs")
    title_row.append("aruco_X_vals")
    title_row.append("aruco_Y_vals")

    title_row.append("left_pupil_x")
    title_row.append("left_pupil_y")
    title_row.append("left_pupil_width")
    title_row.append("left_pupil_height")
    title_row.append("left_pupil_angle")

    title_row.append("right_pupil_x")
    title_row.append("right_pupil_y")
    title_row.append("right_pupil_width")
    title_row.append("right_pupil_height")
    title_row.append("right_pupil_angle")

    with open(output_path, 'wb+') as csvfile:
        gaze_data_writer = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
        gaze_data_writer.writerow(title_row)

        data_stream = LiveDataStream()

        raw_input("[INFO:] Press Enter to record gaze data for " + str(recording_time) + " seconds.")

        start_time = time.time()
        while (True):
            new_data_vector = data_stream.read()
            # Parse and write.
            new_csv_row = transform_data_vector(new_data_vector)
            gaze_data_writer.writerow(new_csv_row)

            cur_time = time.time()
            if cur_time - start_time > recording_time:
                break
