from math import pow, sqrt
import pdb

# Class that we can update with gaze data to detect when the user is staring at
# something.
class StareDetector():

    TIMESTAMP_TO_SECONDS = 1000.0

    def __init__(self, trigger_radius=100.0, activation_time=3.0):
        # Not tracking anything at first.
        self.tracking_ID = None
        self.tracking_timestep = None
        # Parameters of stare detection.
        self.trigger_radius = trigger_radius
        self.activation_time = activation_time

    # Update state with gaze data at a new timestep, and return the ID of the
    # marker we think the user us staring at (or None if nothing is being
    # stared at currently).
    # NOTE: This is the only method that should really be considered "public".
    def check_if_staring(self, data_vector):
        # Extract fields from data vector.
        timestep = data_vector.timestamp
        gaze_x = data_vector.gaze_x
        gaze_y = data_vector.gaze_y
        aruco_IDs = data_vector.aruco_IDs
        aruco_X_vals = data_vector.aruco_X_vals
        aruco_Y_vals = data_vector.aruco_Y_vals

        # We're tracking something, so check if the current gaze is still within
        # radius of that.
        if self.tracking_ID:
            if self.check_maintain_stare(gaze_x, gaze_y, aruco_IDs, aruco_X_vals, aruco_Y_vals):
                seconds_tracked = (timestep - self.tracking_timestep) / StareDetector.TIMESTAMP_TO_SECONDS
                print(seconds_tracked)
                if (seconds_tracked >= self.activation_time):
                    return self.tracking_ID
            else:
                self.tracking_ID = None
                self.tracking_timestep = None
        # Otherwise, find something new to track if we can.
        else:
            new_tracking_ID = self.get_new_tracking_ID(gaze_x, gaze_y, aruco_IDs, aruco_X_vals, aruco_Y_vals)
            if (new_tracking_ID):
                self.tracking_ID = new_tracking_ID
                self.tracking_timestep = timestep

    # Check if the user's gaze is still within radius of the marker currently
    # being tracked.
    def check_maintain_stare(self, gaze_x, gaze_y, aruco_IDs, aruco_X_vals, aruco_Y_vals):
        if self.tracking_ID not in aruco_IDs:
            # TODO: Sometimes the marker drops off because of detection issues.
            # Might want to think about what we should actually do here.
            return False

        tracked_marker_index = aruco_IDs.index(self.tracking_ID)
        current_marker_x = aruco_X_vals[tracked_marker_index]
        current_marker_y = aruco_Y_vals[tracked_marker_index]

        return self.check_gaze_in_radius(gaze_x, gaze_y, current_marker_x, current_marker_y)

    # Of the markers detected by the headset, return the ID (if there is one) of
    # a marker the user's gaze is currently within radius of. We will then start
    # tracking that marker.
    def get_new_tracking_ID(self, gaze_x, gaze_y, aruco_IDs, aruco_X_vals, aruco_Y_vals):
        for i in range(len(aruco_IDs)):
            potential_ID = aruco_IDs[i]
            if self.check_gaze_in_radius(gaze_x, gaze_y, aruco_X_vals[i], aruco_Y_vals[i]):
                return potential_ID

        return None

    # Simply checks if gaze coordinate is within radius of a marker location.
    def check_gaze_in_radius(self, gaze_x, gaze_y, aruco_x, aruco_y):
        x_diff = gaze_x - aruco_x
        y_diff = gaze_y - aruco_y
        distance = sqrt(pow(x_diff, 2) + pow(y_diff, 2))
        return distance <= self.trigger_radius
