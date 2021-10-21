import json
import leveldb_to_hdf5
import normalize_labels
import shift_axes

if __name__ == "__main__":
    f = open("preprocessing_config.json")
    config = json.loads(f.read())
    LEVELDB_PATH = config["leveldb_path"] 
    HDF5_PATH = config["hdf5_path"]
    RELATIVE_TEST_SIZE = config["relative_test_size"]
    DEBUGGING_DATASET = config["debugging_dataset"]
    CONVERSION_BATCH_SIZE = config["conversion_batch_size"]

    ANGLE_RANGE = config["angle_range"]
    TOMARKING_L_RANGE = config["toMarking_L_range"]
    TOMARKING_M_RANGE = config["toMarking_M_range"]
    TOMARKING_R_RANGE = config["toMarking_R_range"]
    DIST_L_RANGE = config["dist_L_range"]
    DIST_R_RANGE = config["dist_R_range"]
    TOMARKING_LL_RANGE = config["toMarking_LL_range"]
    TOMARKING_ML_RANGE = config["toMarking_ML_range"]
    TOMARKING_MR_RANGE = config["toMarking_MR_range"]
    TOMARKING_RR_RANGE = config["toMarking_RR_range"]
    DIST_LL_RANGE = config["dist_LL_range"]
    DIST_MM_RANGE = config["dist_MM_range"]
    DIST_RR_RANGE = config["dist_RR_range"]
    FAST_RANGE = config["fast_range"]
    f.close()

    LABEL_RANGE = [ANGLE_RANGE, TOMARKING_L_RANGE, TOMARKING_M_RANGE, TOMARKING_R_RANGE, \
    DIST_L_RANGE, DIST_R_RANGE, TOMARKING_LL_RANGE, TOMARKING_ML_RANGE, TOMARKING_MR_RANGE, \
    TOMARKING_RR_RANGE, DIST_LL_RANGE, DIST_MM_RANGE, DIST_RR_RANGE, FAST_RANGE]

    print("preprocessing takes a long time (approx. 1h)")
    #leveldb_to_hdf5.leveldb_to_hdf5(level_db_path=LEVELDB_PATH, hdf5_path=HDF5_PATH, \
    #    relative_test_size=RELATIVE_TEST_SIZE, conversion_batch_size=CONVERSION_BATCH_SIZE)

    shift_axes.shift_axes(hdf5_path=HDF5_PATH)

    normalize_labels.normalize_dataset_labels(hdf5_path=HDF5_PATH, label_range=LABEL_RANGE)

