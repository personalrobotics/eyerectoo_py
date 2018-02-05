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

import numpy as np


class dataFilter() :

	# take in a data stream (could be csv_data_stream or live_data_stream)
	def __init__(self) : # could have passed in datastream, but decided against it
		# self.data_stream = datastream
		self.num_frames_erase = 10 # erase 10 frames at once for efficiency
		self.frames_memory = 50 + self.num_frames_erase # remember only the last 50 values (up to 60)
		self.gaze_vector = []
		# self.gaze_valid = [] # made gaze_valid a field in gaze_data_vector

		self.min_window_size = 40 # for the pupil values
		self.right_pupil_widths = [] # keep window of recent pupil values
		self.right_pupil_heights = []
		self.left_pupil_widths = []
		self.left_pupil_heights = []

		self.max_std = 20 # use windows that have max 20 pixel standard deviation for pupil size estimation
		self.filtered_right_pupil_widths = [] # tuples of avg values and stdevs
		self.filtered_right_pupil_heights = []
		self.filtered_left_pupil_widths = []
		self.filtered_left_pupil_heights = []


	def set_gaze_valid_field(self, new_gaze_vector) :
		# get a new frame
		self.gaze_vector.append(new_gaze_vector)
		# do the filtering on the new frame
		frame_valid = self.filter_gaze_data_vector(new_gaze_vector)
		# self.gaze_valid.append(frame_valid)

		# if we've stored too many frames, let's remove some to maintain memory...
		if (len(self.gaze_vector) > self.frames_memory) :
			del self.gaze_vector[:self.num_frames_erase] # remove first n frames
		# if (len(self.gaze_valid) > self.frames_memory) :
		# 	del self.gaze_valid[:self.num_frames_erase] # remove first n frames

		# return frame_valid
		return self.gaze_vector[-1]


	# do the filtering in here
	def filter_gaze_data_vector(self, curr_gaze_vector) :
		# each of these filtering functions returns a bool
		if (self.filter_field_width_height(curr_gaze_vector)) :
			if len(self.gaze_vector) > 2 : # this filter only works with 2 data pts
				# if (self.filter_pupil_width_height(curr_gaze_vector)) :
				if (self.filter_pupil_size(curr_gaze_vector)) :
					return True
					# return curr_gaze_vector
		self.gaze_vector[-1].gaze_valid = False
		return False
		# return self.gaze_vector[-1]


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


	# filters pupil width/height based on a best guess of the real pupil size
	# if we don't have enough data points for this method, use filter_pupil_width_height
	def filter_pupil_size(self, curr_gaze_vector) :
		if len(self.gaze_vector) >= self.min_window_size : # min 40 points to begin windowing data...
			v1 = self.helper_filter_pupil_size(self.right_pupil_widths, self.filtered_right_pupil_widths, curr_gaze_vector.right_pupil_width)
			v2 = self.helper_filter_pupil_size(self.right_pupil_heights, self.filtered_right_pupil_heights, curr_gaze_vector.right_pupil_height)
			v3 = self.helper_filter_pupil_size(self.left_pupil_widths, self.filtered_left_pupil_widths, curr_gaze_vector.left_pupil_width)
			v4 = self.helper_filter_pupil_size(self.left_pupil_widths, self.filtered_left_pupil_heights, curr_gaze_vector.left_pupil_height)

			if (v1 and v2 and v3 and v4) :
				return True
			else :
				return False

		else : # we just don't have enough points...
			return self.filter_pupil_width_height(curr_gaze_vector)


	def helper_filter_pupil_size(self, pupil_vector, good_pupil_vals, curr_val) :
		thresh = 20 # within 20 pixels of the pupil size we expect

		# first add current value
		pupil_vector.append(curr_val)

		# calculate over window the avg and stdev
		windowed_avg = np.mean(pupil_vector[-1*self.min_window_size:])
		windowed_std = np.std(pupil_vector[-1*self.min_window_size:])
		if (windowed_std <= self.max_std) :
			good_pupil_vals.append([windowed_avg, windowed_std])

		# let's manage some memory - don't want to store too too much
		if (len(good_pupil_vals) > self.frames_memory) :
			del good_pupil_vals[:self.num_frames_erase]

		# if we have good values, use them to check current gaze vector
		if (len(good_pupil_vals) > 0) :
			# calculate the pupil value
			# calculated_pupil_val = np.average(good_pupil_vals[:,0], weights=( np.asarray(good_pupil_vals)[:,1]) # use std as weights -- no bueno! should be HIGHER weights for lower stdevs
			calculated_pupil_val = np.average(np.asarray(good_pupil_vals)[:,0])
			if (curr_val < calculated_pupil_val + thresh) and (curr_val > calculated_pupil_val - thresh) :
				return True
			else :
				return False
		# else if we don't have any good values yet, use the old method...
		else :
			return filter_pupil_width_height(curr_gaze_vector)




	# checks whether the pupil width/height have jumped in value beyond the threshold in the past frame
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
