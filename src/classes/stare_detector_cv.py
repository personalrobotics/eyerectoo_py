from math import pow, sqrt
import pdb

# Class that we can update with gaze data to detect when the user is staring at
# something (with some CV detector in the loop!)
class StareDetectorCV():

    TIMESTAMP_TO_SECONDS = 1000.0

    def __init__(self, activation_time=3.0, bb_persistance_iterations=10):
        # Not tracking anything at first.
        self.tracking_ID = None
        self.tracking_timestep = None
        # Parameters of stare detection.
        self.activation_time = activation_time
        self.bb_persistance_iterations = bb_persistance_iterations
        self.elapsed_persistance_iterations = 0


    # Update state with gaze data and object detections at a new timestep.
    # Return the name of the object we think the user is staring at (or None if
    # nothing is being stared at currently).
    # NOTE: This is the only method that should really be considered "public".
    def check_if_staring(self, data_vector, detections):
        # Extract fields from data vector.
        timestep = data_vector.timestamp
        gaze_x = data_vector.gaze_x
        gaze_y = data_vector.gaze_y

        # We're tracking something, so check if the current gaze is still within
        # radius of that.
        if self.tracking_ID:
            if self.check_maintain_stare(gaze_x, gaze_y, detections):
                seconds_tracked = (timestep - self.tracking_timestep) / StareDetectorCV.TIMESTAMP_TO_SECONDS
                print(seconds_tracked)
                if (seconds_tracked >= self.activation_time):
                    return self.tracking_ID
            else:
                self.tracking_ID = None
                self.tracking_timestep = None
        # Otherwise, find something new to track if we can.
        else:
            new_tracking_ID = self.get_new_tracking_ID(gaze_x, gaze_y, detections)
            if (new_tracking_ID):
                self.tracking_ID = new_tracking_ID
                self.tracking_timestep = timestep

    # Check if the user's gaze is still intersecting the BB of the object
    # currently being tracked.
    def check_maintain_stare(self, gaze_x, gaze_y, detections):
        detected_objects = [i[0] for i in detections]
        if self.tracking_ID not in detected_objects:
            # Add some persistance to the BB to account for intermittant
            # detections.
            self.elapsed_persistance_iterations += 1
            if (self.elapsed_persistance_iterations == self.bb_persistance_iterations):
                self.elapsed_persistance_iterations = 0
                self.object_bb = None
                return False
            else:
                tracked_object_data = (self.tracking_ID, 0.5, self.object_bb)
        else:
            tracked_object_index = detected_objects.index(self.tracking_ID)
            tracked_object_data = detections[tracked_object_index]

        return self.check_gaze_intersection(gaze_x, gaze_y, tracked_object_data)

    # Of the objects detected by the NN, return the name (if there is one) of
    # an object the user's gaze is currently within radius of. We will then start
    # tracking that object.
    def get_new_tracking_ID(self, gaze_x, gaze_y, detections):
        for cur_detection in detections:
            potential_object_name = cur_detection[0]
            if self.check_gaze_intersection(gaze_x, gaze_y, cur_detection):
                self.object_bb = cur_detection[2]
                return potential_object_name

        return None

    # Simply checks if gaze coordinate is within bounding box of a detected
    # object.
    def check_gaze_intersection(self, gaze_x, gaze_y, object_detection):
        center_x = object_detection[2][0]
        center_y = object_detection[2][1]

        bb_width = object_detection[2][2]
        bb_hieght = object_detection[2][3]

        bb_upper_left_x = center_x - (bb_width/2)
        bb_upper_left_y = center_y - (bb_hieght/2)

        bb_lower_right_x = center_x + (bb_width/2)
        bb_lower_right_y = center_y + (bb_hieght/2)

        in_bb = (gaze_x > bb_upper_left_x and gaze_y > bb_upper_left_y) and (gaze_x < bb_lower_right_x and gaze_y < bb_lower_right_y)
        return in_bb
