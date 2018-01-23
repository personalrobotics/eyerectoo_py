"""
Morelle Arian
Firstly, the 20 second recordings don't start at the beginning of the file.
Gotta search for the starting index (there's a jump in timestamp -- i searched for jump of over 1000 ms)

Then, I started filtering the data by looking for neighboring spikes
Any value that jumps too fast is masked out of plotting.
Helps smooth the clean data well.
	Limitations: if you really are moving your eyes in a crazy fashion, we may lose it
	On the other hand, we will catch sudden movements that stay at a destination
	(because the destination's neighboring values will be simialr)
Worked really well on my own clean data.
Worked reasonably well on Sherdil's but there IS one data point that had a 2 point spike that wasn't filtered.

Look at comment below for future plan... Stefanos's data is BAD. Needs major filtering
"""

import pandas
from matplotlib import pyplot
from scipy.signal import butter, lfilter
from scipy.signal import freqs
import numpy as np

# data = pandas.read_csv("recordings/morelle_clean_data.csv")
# data = pandas.read_csv("recordings/sherdil_clean_data.csv")
data = pandas.read_csv("recordings/stefanos_dirty_data.csv")

"""
OK - stefanos's data is REALLY bad. what's the resolution of the image? his data
goes over 80000 so whatever the height/width of the image is, we can at minimum
throw out all the points outside of the image. (?)

	field_width & field_height are the values i'm looking for
whoah in his data some of the valid signals are False!
but the overall valid signal doesn't correspond - wonder if they did what i did and ORed instead of ANDed

pupil_width & pupil_height we can also filter. both for 0 (blinks <-- sherdil) and for too large (iris)

side note: i'd also like to plot aruco values to see how stable they are
"""

#for i in dat:
#   print(i)

"""
timestamp
gaze_x
gaze_y
valid_gaze
field_width
field_height
aruco_markers_present
aruco_IDs
aruco_X_vals
aruco_Y_vals
left_pupil_x
left_pupil_y
left_pupil_width
left_pupil_height
left_pupil_angle
left_pupil_valid
right_pupil_x
right_pupil_y
right_pupil_width
right_pupil_height
right_pupil_angle
right_pupil_valid
"""

# would like to find the place where the difference between index and neighbor is greater than a threshold
def compare_neighbor(arr, diff) :
	for i in range(len(arr)-1) :
		if arr[i+1]-arr[i] > diff : # should be strictly increasing because it's time
			return i+1
	return -1 # didn't find anything


# taken from https://stackoverflow.com/questions/25191620/creating-lowpass-filter-in-scipy-understanding-methods-and-units
def butter_lowpass(cutOff, fs, order=5):
    nyq = 0.5 * fs
    normalCutoff = cutOff / nyq
    b, a = butter(order, normalCutoff, btype='low', analog = True)
    return b, a

def butter_lowpass_filter(data, cutOff, fs, order=4):
    b, a = butter_lowpass(cutOff, fs, order=order)
    y = lfilter(b, a, data)
    return y

# average values within a window to emulate a lowpass filter
def averaging_filter(data, window) :
	new_data = []
	for i in range(len(data)-window) :
		new_data.append(np.mean(data[i:i+window]))
	return new_data


def plot_gaze(t, gaze_x, gaze_y) :
	# plot x-gaze & y-gaze over time (in seconds)
	# pyplot.title("gaze")
	pyplot.subplot(211)
	# pyplot.title("x-gaze")
	pyplot.xlabel("time, seconds")
	# pyplot.ylabel("gaze, pixels")
	pyplot.plot(t, gaze_x)
	pyplot.subplot(212)
	# pyplot.title("y-gaze")
	pyplot.xlabel("time, seconds")
	# pyplot.ylabel("gaze, pixels")
	pyplot.plot(t, gaze_y)
	# pyplot.show()

# pass in boolean mask with True/False values and plot only points that have True values
def plot_gaze_mask(t, gaze_x, gaze_y, mask) :
	# plot x-gaze & y-gaze over time (in seconds)
	# pyplot.title("gaze")
	pyplot.subplot(211)
	# pyplot.title("x-gaze")
	pyplot.xlabel("time, seconds")
	# pyplot.ylabel("gaze, pixels")
	pyplot.plot(t[mask], gaze_x[mask])
	pyplot.subplot(212)
	# pyplot.title("y-gaze")
	pyplot.xlabel("time, seconds")
	# pyplot.ylabel("gaze, pixels")
	pyplot.plot(t[mask], gaze_y[mask])
	# pyplot.show()

# pass in boolean mask with True/False values and plot only points that have True values
# but also plot the 0/1 values themselves!
def plot_gaze_mask2(t, gaze_x, gaze_y, mask) :
	# plot x-gaze & y-gaze over time (in seconds)
	# pyplot.title("gaze")
	pyplot.subplot(311)
	# pyplot.title("x-gaze")
	pyplot.xlabel("time, seconds")
	# pyplot.ylabel("gaze, pixels")
	pyplot.plot(t[mask], gaze_x[mask])
	pyplot.subplot(312)
	# pyplot.title("y-gaze")
	pyplot.xlabel("time, seconds")
	# pyplot.ylabel("gaze, pixels")
	pyplot.plot(t[mask], gaze_y[mask])
	pyplot.subplot(313)
	pyplot.plot(t, mask*2)
	pyplot.ylim(-0.5, 2.5)
	# pyplot.show()


# i think the recording starts partway through actually. let's search for beginning
time = data["timestamp"]-data["timestamp"][0]
start_index = compare_neighbor(time, 1000)
print("start index in time: " + str(start_index))
time = time/1000 # change from ms to s
time = time-time[start_index] # let's re-zero start-time

# plot_gaze(time[start_index:], data["gaze_x"][start_index:], data["gaze_y"][start_index:])


# now, basic validation, look for values outside of our field_width & field_height
gaze_x_valid = np.logical_and(np.invert(data["gaze_x"] > data["field_width"]), np.invert(data["gaze_x"] < 0))
gaze_y_valid = np.logical_and(np.invert(data["gaze_y"] > data["field_height"]), np.invert(data["gaze_y"] < 0))
valid_gaze = np.logical_and(gaze_x_valid, gaze_y_valid)
# print(np.where(valid_gaze == False))


# # oh yeah, and let's add in the left/right_pupil_valid -- appanretly some are false!
# # i think this signal might be shit! gets rid of some points that look fine...
# valid_gaze = np.logical_and(valid_gaze, data["left_pupil_valid"])
# valid_gaze = np.logical_and(valid_gaze, data["right_pupil_valid"])

# pyplot.figure("gaze with FIELD width/height validation")
# plot_gaze_mask2(time[start_index:], data["gaze_x"][start_index:], data["gaze_y"][start_index:], valid_gaze[start_index:])
# pyplot.show()

# # valid_gaze is all just set to True... useless! -- found some datasets where it IS False!!
# valid_gaze = np.asarray(data["valid_gaze"])
# print(type(valid_gaze[0]))
# print(np.all(valid_gaze))



# ----------------------------------------------------------------

# # NOW let's look at pupil_width & pupil_height
# pyplot.figure("PUPIL width/height")
# plot_gaze_mask(time[start_index:], data["left_pupil_width"][start_index:], data["left_pupil_height"][start_index:], valid_gaze[start_index:])
# pyplot.show()
#
# # let's try and remove spikes again ?
# pyplot.title("abs val of diff between neighbors in pupil width & height")
# pyplot.subplot(211)
# pyplot.title("pupil width")
# pyplot.plot(abs(np.diff(data["left_pupil_width"][start_index-1:])))
# pyplot.subplot(212)
# pyplot.title("pupil height")
# pyplot.plot(abs(np.diff(data["left_pupil_height"][start_index-1:])))
# pyplot.show()

# remove spikes in pupil_width & pupil_height.
# look at first graph above to persuade you this is a good idea. look at second graph for threshold.
threshold = 10 # spikes larger than 10 pixels in 2ms are thrown out
left_pupil_width_valid = np.invert(abs(np.diff(data["left_pupil_width"][start_index-1:])) > threshold)
left_pupil_height_valid = np.invert(abs(np.diff(data["left_pupil_height"][start_index-1:])) > threshold)
left_pupil_WH_valid = np.logical_and(left_pupil_width_valid, left_pupil_height_valid)
# pyplot.figure("left pupil width & height NO filtering")
# plot_gaze(time[start_index:], data["left_pupil_width"][start_index:], data["left_pupil_height"][start_index:])
# pyplot.figure("left pupil width & height WITH filtering")
# plot_gaze_mask(time[start_index:], data["left_pupil_width"][start_index:], data["left_pupil_height"][start_index:], left_pupil_WH_valid)
# pyplot.show()

## HERE YOU SEE HOW BAD STEFANOS' DATA IS. IT'S BECAUSE OF RIGHT PUPIL!
right_pupil_width_valid = np.invert(abs(np.diff(data["right_pupil_width"][start_index-1:])) > threshold)
right_pupil_height_valid = np.invert(abs(np.diff(data["right_pupil_height"][start_index-1:])) > threshold)
right_pupil_WH_valid = np.logical_and(right_pupil_width_valid, right_pupil_height_valid)



# i think what's happening is that the pupil width/height = 0 where it's valid...
# still not sure why the second graph looks so much better than everything else.
"""
this is a sign to filter out all the values = 0
Not necessarily helpful - the algorithm autamically does something with blinks i think
"""
# i don't think this is necessary but it is useful to visualize....
right_pupil_width_nonzero = np.invert(abs(np.diff(data["right_pupil_width"][start_index-1:])) == 0)
right_pupil_height_nonzero = np.invert(abs(np.diff(data["right_pupil_height"][start_index-1:])) == 0)
right_pupil_WH_nonzero = np.logical_and(right_pupil_width_nonzero, right_pupil_height_nonzero)

right_pupil_combined_valid = np.logical_and(right_pupil_WH_valid, right_pupil_WH_nonzero)

# # # uncomment this to see cleaning of stefanos' signal.
# # pyplot.figure("right pupil width & height NO filtering")
# # plot_gaze(time[start_index:], data["right_pupil_width"][start_index:], data["right_pupil_height"][start_index:])
# pyplot.figure("right pupil width & height WITH filtering")
# plot_gaze_mask2(time[start_index:], data["right_pupil_width"][start_index:], data["right_pupil_height"][start_index:], right_pupil_WH_valid)
# # pyplot.show()
#
# # pyplot.figure("original signal")
# # plot_gaze(time[start_index:], data["gaze_x"][start_index:], data["gaze_y"][start_index:])
# pyplot.figure("signal with right pupil width & height filtering")
# plot_gaze_mask2(time[start_index:], data["gaze_x"][start_index:], data["gaze_y"][start_index:], right_pupil_WH_valid)
# # pyplot.show()

# # THIS IS NOT ENOUGH FILTERING...
# pyplot.figure("original signal valid gaze")
# plot_gaze(time[start_index:], data["gaze_x"][start_index:], data["gaze_y"][start_index:])
# pyplot.figure("signal with valid gaze filtering")
# plot_gaze_mask(time[start_index:], data["gaze_x"][start_index:], data["gaze_y"][start_index:], valid_gaze[start_index:])
# pyplot.show()

# print( str(len(np.where(right_pupil_WH_valid == False)[0])) + " bad data pts in right pupil height & width")
# print( str(len(np.where(valid_gaze[start_index:] == False)[0])) + " other bad pts in valid_gaze")


# ----------------------------------------------------------------

# remove spikes in pupil_x and pupil_y
# might not actually be necessary. would also remove saccades which is no bueno.
threshold = 6 # spikes larger than 6 pixels in 2ms are thrown out
right_pupil_valid_x = np.invert(abs(np.diff(data["right_pupil_x"][start_index-1:])) > threshold)
right_pupil_valid_y = np.invert(abs(np.diff(data["right_pupil_y"][start_index-1:])) > threshold)
right_pupil_valid = np.logical_and(right_pupil_valid_x, right_pupil_valid_y)
# print(np.where(right_pupil_valid == False))

# # plot the masked values only! THIS IS ALSO WHY STEFANOS' DATA IS SO BAD
# pyplot.figure("right pupil x & y original")
# plot_gaze(time[start_index:], data["right_pupil_x"][start_index:], data["right_pupil_y"][start_index:])
# pyplot.figure("right pupil x & y masked")
# plot_gaze_mask(time[start_index:], data["right_pupil_x"][start_index:], data["right_pupil_y"][start_index:], right_pupil_valid)
# pyplot.show()

# looks reasonable! let's do the same with left pupil and plot the gaze values in total is valid
left_pupil_valid_x = np.invert(abs(np.diff(data["left_pupil_x"][start_index-1:])) > threshold)
left_pupil_valid_y = np.invert(abs(np.diff(data["left_pupil_y"][start_index-1:])) > threshold)
left_pupil_valid = np.logical_and(left_pupil_valid_x, left_pupil_valid_y)
# # print(np.where(left_pupil_valid == False))
# # plot the masked values only!
# pyplot.figure("left pupil x & y masked")
# plot_gaze_mask(time[start_index:], data["left_pupil_x"][start_index:], data["left_pupil_y"][start_index:], left_pupil_valid)
# pyplot.show()

# ----------------------------------------------------------------
# after looking at both the pupil height/width values and the x/y values,
# i think that the height/width works better as a filter (looking for spikes)
# they may be a symptom of the same issue (stefanos' right eye was not tracking well)
# but this former filtering works better! earlier in the data analysis

# hmm but they might actually both be equally bad.
# i think we just need to take better data for stefanos...
# i do think that the x/y values ends up filtering more. definitely true for my file (morelle_clean_data)

# let's make valid_gaze shorter now!
valid_gaze = valid_gaze[start_index:]

# valid_gaze = np.logical_and(left_pupil_valid, valid_gaze)
# valid_gaze = np.logical_and(right_pupil_valid, valid_gaze)

# i like leaving this one in (not the one above).
valid_gaze = np.logical_and(left_pupil_WH_valid, valid_gaze)
valid_gaze = np.logical_and(right_pupil_WH_valid, valid_gaze)

# print(np.where(valid_gaze == False))

pyplot.figure("filtered")
plot_gaze_mask2(time[start_index:], data["gaze_x"][start_index:], data["gaze_y"][start_index:], valid_gaze)
pyplot.figure("original signal")
plot_gaze(time[start_index:], data["gaze_x"][start_index:], data["gaze_y"][start_index:])
pyplot.show()

# maybe for stefanos' data, it's sufficient to say that too many points were invalid...
print( str(len(np.where(valid_gaze == False)[0])) + " data points were thrown out out of " + str(len(valid_gaze)) )
print("suggestion: reject out when more than X percent the data points are invalid")



"""
plot shows pretty stable readings - within 25 pixels in both x- and y- directions
a few spikes ~ 3 in each direction. they're not always the same time in each direction
2 things to do : look at whether there's a reason spike happened (blinks?)
lowpass filter <-- this will be simplest because if this issue is because of opencv
then it'll still take care of it.
"""
# fs = (time[len(time)-1]-time[start_index])/(len(time)-start_index)
# fs = pow(fs, -1) # convert into hz; i had the fraction opposite above...
# print("sampling freq: " + str(fs))
# cutoff_freq = 1 # 60 hz?
# x_filt = butter_lowpass_filter(data["gaze_x"], cutoff_freq, fs)
# y_filt = butter_lowpass_filter(data["gaze_y"], cutoff_freq, fs)
# plot_gaze(time[start_index:], x_filt[start_index:], y_filt[start_index:])
"""huh. doesn't seem to be working all that well... not sure why but something
about the cutoff frequency is funky. i can't get this to get rid of
just the super high frequency noise. ? maybe something simpler. just averaging"""

# window = 3
# x_filt = averaging_filter(data["gaze_x"], window)
# y_filt = averaging_filter(data["gaze_y"], window)
# print(len(x_filt))
# print(len(data["gaze_x"]))
# plot_gaze(time[start_index:], x_filt[start_index-window:], y_filt[start_index-window:])

"""
let's check out how good the pupil x,y coords are
"""
# plot_gaze(time[start_index:], data["left_pupil_x"][start_index:], data["left_pupil_y"][start_index:])
# plot_gaze(time[start_index:], data["right_pupil_x"][start_index:], data["right_pupil_y"][start_index:])
# yep! right pupil is bad. we just gotta be able to detect this spikes and invalidate that data
# let's try and set right_pupil_valid ourselves.
## for future, grep for ellipse in the code and see if that function has ANY kind of confidence interval

# # plot the difference between neighboring points -- we expect to see spikes where our data is bad
# left_pupil_valid_x = abs(np.diff(data["left_pupil_x"][start_index-1:]))
# left_pupil_valid_y = abs(np.diff(data["left_pupil_y"][start_index-1:]))
# pyplot.subplot(211)
# pyplot.plot(time[start_index:], left_pupil_valid_x)
# pyplot.subplot(212)
# pyplot.plot(time[start_index:], left_pupil_valid_y)
"""the only place this doesn't look good on sherdil's graph is where the spike is for TWO pts not just one"""
