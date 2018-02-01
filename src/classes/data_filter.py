"""
Takes a csv_data_stream (or later a live_data_stream) & filters the data.
Should later pass the filtered data to the stare_detector.
	Note that the stare_detector should also have a lowpass filter (so that it
	doesn't just lose the gaze if one data point is off even with the filtering)

Filtering:
 - * checks whether the gaze values are out of range of field width & height
 - * checks whether the pupil width/height have jumped in value beyond the threshold in the past frame
 - remove spikes in pupil_x and pupil_y
 - remove points where pupil width or height are zero (= blinks)
* currently being checked in filter() function... the others are never called

TODO:
- make the pupil width/height estimate mo betta
- get stare detector working
"""




class dataFilter() :

	# take in a data stream (could be csv_data_stream or live_data_stream)
	def __init__(self) : # could have passed in datastream, but decided against it
		# self.data_stream = datastream
		self.num_frames_erase = 10 # erase 10 frames at once for efficiency
		self.frames_memory = 50 + self.num_frames_erase # remember only the last 50 values (up to 60)
		self.gaze_vector = []
		self.gaze_valid = []


	def filter(self, new_gaze_vector) :
		# get a new frame
		self.gaze_vector.append(new_gaze_vector)
		# do the filtering on the new frame
		frame_valid = self.filter_gaze_data_vector(new_gaze_vector)
		self.gaze_valid.append(frame_valid)

		# if we've stored too many frames, let's remove some to maintain memory...
		if (len(self.gaze_vector) > self.frames_memory) :
			del self.gaze_vector[:self.num_frames_erase] # remove first n frames
		if (len(self.gaze_valid) > self.frames_memory) :
			del self.gaze_valid[:self.num_frames_erase] # remove first n frames

		return frame_valid


	# do the filtering in here
	def filter_gaze_data_vector(self, curr_gaze_vector) :
		# each of these filtering functions returns a bool
		if (self.filter_field_width_height(curr_gaze_vector)) :
			if len(self.gaze_vector) > 2 : # this filter only works with 2 data pts
				if (self.filter_pupil_width_height(curr_gaze_vector)) :
					return True
		return False


	# checks whether the gaze values are out of range of field width & height
	def filter_field_width_height(self, curr_gaze_vector) :
		# now, basic validation, look for values outside of our field_width & field_height
		gaze_x_valid = (not curr_gaze_vector.gaze_x > curr_gaze_vector.field_width) and (not curr_gaze_vector.gaze_x < 0)
		gaze_y_valid = (not curr_gaze_vector.gaze_y > curr_gaze_vector.field_height) and (not curr_gaze_vector.gaze_y < 0)
		valid_gaze = gaze_x_valid and gaze_y_valid
		# gaze_x_valid = np.logical_and(np.invert(curr_gaze_vector.gaze_x > curr_gaze_vector.field_width), np.invert(curr_gaze_vector.gaze_x < 0))
		# gaze_y_valid = np.logical_and(np.invert(curr_gaze_vector.gaze_y > curr_gaze_vector.field_height), np.invert(curr_gaze_vector.gaze_y < 0))
		# valid_gaze = np.logical_and(gaze_x_valid, gaze_y_valid)
		return valid_gaze


	# checks whether the pupil width/height have jumped in value beyond the threshold in the past frame
### we should really do a better check here because we can find pupil size
	# If the spike remains high, we just lose one data point
	# If the spike goes back, we lose two data points but remove the dirty point
	def filter_pupil_width_height(self, curr_gaze_vector) :
		# remove spikes in pupil_width & pupil_height.
		threshold = 10 # spikes larger than 10 pixels in 2ms are thrown out
		prev_gaze_vector = self.gaze_vector[-2]
		if (abs(prev_gaze_vector.left_pupil_width - curr_gaze_vector.left_pupil_width) > threshold) :
			return False
		if (abs(prev_gaze_vector.left_pupil_height - curr_gaze_vector.left_pupil_height) > threshold) :
			return False
		if (abs(prev_gaze_vector.right_pupil_width - curr_gaze_vector.right_pupil_width) > threshold) :
			return False
		if (abs(prev_gaze_vector.right_pupil_height - curr_gaze_vector.right_pupil_height) > threshold) :
			return False
		return True


	# remove spikes in pupil_x and pupil_y
	# might not actually be necessary. would also remove saccades which is no bueno.
	def filter_pupil_x_y(self, curr_gaze_vector):
		threshold = 6 # spikes larger than 6 pixels in 2ms are thrown out
		prev_gaze_vector = self.gaze_vector[-2]
		if abs(prev_gaze_vector.right_pupil_x - curr_gaze_vector.right_pupil_x) > threshold :
			return False
		if abs(prev_gaze_vector.right_pupil_y - curr_gaze_vector.right_pupil_y) > threshold :
			return False
		if abs(prev_gaze_vector.left_pupil_x - curr_gaze_vector.left_pupil_x) > threshold :
			return False
		if abs(prev_gaze_vector.left_pupil_y - curr_gaze_vector.left_pupil_y) > threshold :
			return False
		return True


	# remove points where pupil width or height are zero (= blinks)
	# i don't think this is necessary but it is useful to visualize....
	# leaving it in in case somebody wants this
	def filter_blinks(self, curr_gaze_vector) :
		if (curr_gaze_vector.right_pupil_width == 0) :
			return False
		if (curr_gaze_vector.right_pupil_height == 0) :
			return False
		if (curr_gaze_vector.left_pupil_width == 0) :
			return False
		if (curr_gaze_vector.left_pupil_height == 0) :
			return False
		return True
