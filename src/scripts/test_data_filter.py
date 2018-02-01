# in order to run:
# python test_data_filter.py "../../filtering/recordings/morelle_clean_data.csv"

#!/usr/bin/env python
import sys
# Hack to make the next imports work.
sys.path.append("../classes/")
from csv_data_stream import CSVDataStream
from stare_detector import StareDetector
from data_filter import dataFilter
import pdb
from matplotlib import pyplot

if __name__ == "__main__":

    csv_path = sys.argv[1]

    data_stream = CSVDataStream(csv_path)
    filt = dataFilter()
    stare_detector = StareDetector()
    valid_vector = []

    while (True):
        new_data_vector = data_stream.read()
        # Data stream will return None when reading past end of CSV.
        if (new_data_vector == None):
            break
        # otherwise, let's filter the data!
        valid = filt.filter(new_data_vector)
        valid_vector.append(valid)

        if (valid) :
            stare_marker_id = stare_detector.check_if_staring(new_data_vector)
            if (stare_marker_id):
                print("STARING AT MARKER " + str(stare_marker_id))
    print(str(len(valid_vector) - sum(valid_vector)) + " points thrown out out of " + str(len(valid_vector)))
    pyplot.plot(valid_vector)
    pyplot.show()
