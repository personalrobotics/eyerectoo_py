#!/usr/bin/env python
import csv
import time
import sys

from eyerectoo_capture import EyeRecTooCapture

import pdb

def parse_journal_data(journal_data):
    ''' Add a line to the CSV from new Journal data.'''
    new_csv_row = []

    # TODO: Should remember in an elegant way which fields are at
    # which index, not have magic numbers like this.

    # Split off the "J"
    sync_timestamp = int(journal_data[0][1:])
    new_csv_row.append(str(sync_timestamp))

    gaze_x = float(journal_data[2])
    gaze_y = float(journal_data[3])
    valid_gaze = int(journal_data[5]) == 1
    new_csv_row.append(str(gaze_x))
    new_csv_row.append(str(gaze_y))
    new_csv_row.append(str(valid_gaze))

    field_width = int(journal_data[11])
    field_height = int(journal_data[12])
    new_csv_row.append(str(field_width))
    new_csv_row.append(str(field_height))

    # Handle Aruco Data
    aruco_markers_present = len(journal_data) > 30
    new_csv_row.append(str(aruco_markers_present))

    aruco_IDs = []
    aruco_X_vals = []
    aruco_Y_vals = []
    if aruco_markers_present:
        aruco_string = journal_data[13]
        # There's an extra semicolon at the end for some reason.
        aruco_parts = aruco_string.split(';')[:-1]
        for tag_data in aruco_parts:
            tag_ID, tag_coords_str = tag_data.split(':')
            tag_x, tag_y, _ = tag_coords_str.split('x')
            aruco_IDs.append(tag_ID)
            aruco_X_vals.append(tag_x)
            aruco_Y_vals.append(tag_y)


        # Remove the aruco marker data to make indicies consistent
        journal_data = journal_data[:13] + journal_data[14:]

        aruco_IDs = ":".join(aruco_IDs)
        aruco_X_vals = ":".join(aruco_X_vals)
        aruco_Y_vals = ":".join(aruco_Y_vals)

        #pdb.set_trace()

    else:
        aruco_IDs = "None"
        aruco_X_vals = "None"
        aruco_Y_vals = "None"

    new_csv_row.append(aruco_IDs)
    new_csv_row.append(aruco_X_vals)
    new_csv_row.append(aruco_Y_vals)
    
    left_pupil_x = float(journal_data[15])
    left_pupil_y = float(journal_data[16])
    left_pupil_width = float(journal_data[17])
    left_pupil_height = float(journal_data[18])
    left_pupil_angle = float(journal_data[19])
    left_pupil_valid = int(journal_data[20]) == 1

    new_csv_row.append(str(left_pupil_x))
    new_csv_row.append(str(left_pupil_y))
    new_csv_row.append(str(left_pupil_width))
    new_csv_row.append(str(left_pupil_height))
    new_csv_row.append(str(left_pupil_angle))
    new_csv_row.append(str(left_pupil_valid))

    right_pupil_x = float(journal_data[23])
    right_pupil_y = float(journal_data[24])
    right_pupil_width = float(journal_data[25])
    right_pupil_height = float(journal_data[26])
    right_pupil_angle = float(journal_data[27])
    right_pupil_valid = int(journal_data[28]) == 1

    new_csv_row.append(str(right_pupil_x))
    new_csv_row.append(str(right_pupil_y))
    new_csv_row.append(str(right_pupil_width))
    new_csv_row.append(str(right_pupil_height))
    new_csv_row.append(str(right_pupil_angle))
    new_csv_row.append(str(right_pupil_valid))

    #pdb.set_trace()

    return new_csv_row


if __name__ == "__main__":

    output_path = sys.argv[1]
    # Number of seconds to record for.
    recording_time = int(sys.argv[2])

    title_row = []
    title_row.append("timestamp")
    title_row.append("gaze_x")
    title_row.append("gaze_y")
    title_row.append("valid_gaze")
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
    title_row.append("left_pupil_valid")

    title_row.append("right_pupil_x")
    title_row.append("right_pupil_y")
    title_row.append("right_pupil_width")
    title_row.append("right_pupil_height")
    title_row.append("right_pupil_angle")
    title_row.append("right_pupil_valid")

    with open(output_path, 'wb+') as csvfile:
        gaze_data_writer = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
        gaze_data_writer.writerow(title_row)

        eyerectoo_listener = EyeRecTooCapture()

        raw_input("[INFO:] Press Enter to record gaze data for " + str(recording_time) + " seconds.")

        start_time = time.clock()
        while (True):
            journal_data = eyerectoo_listener.read() 
            # Parse and write.
            new_csv_row = parse_journal_data(journal_data)
            gaze_data_writer.writerow(new_csv_row)

            cur_time = time.clock()
            if cur_time - start_time > recording_time:
                break