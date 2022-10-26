import logging
import sys
import os

from numpy import True_

from lsdynabin_to_train.tools import *

log_folder = os.environ['LOG_FOLDER']

# Initialize logging
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
file_handler = logging.FileHandler(f'{log_folder}/preprocessing.log')
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
file_handler.setFormatter(formatter)
logger.addHandler(file_handler)

if __name__ == "__main__":
    logger.debug("Start creating h5 from raw...")
    base_dir = "./toolchain/preprocessing/"
    use_cached = False
    get_node_force = False
    get_node_velocity = False
    get_node_acceleration = False
    cache_file = "data_dict.pkl"
    t = range(1,7)
    data_dict = {}
    end = 1 #only first line (1 run only)

    #unpack(base_dir, delete=True)

    graph_dict, invalid_indices = get_d3plots(os.path.join(base_dir, "raw"), end=end, d3plot_range=t, cube_gnn=True)
    invalid_indices = None
    node_forces, node_velocities, node_accelerations, fd = get_force_displacements(os.path.join(base_dir, "raw"), end=end, first_keyword_force="rcforc", force_dir='z_force', disp_dir='y_displacement', invalid_indices=invalid_indices, fd_end=200, custom_disp=np.linspace(0,100,200), get_node_force=get_node_force, get_node_velocity=get_node_velocity, get_node_acceleration=get_node_acceleration, t=t)
    data_dict['fd_label'] = fd
    params = read_params(os.path.join(base_dir, "raw", "Input.csv"), end=end)
    data_dict['params'] = params

    if get_node_force:
        add_feature(graph_dict, node_forces)
    if get_node_velocity:
        add_feature(graph_dict, node_velocities)
    if get_node_acceleration:
        add_feature(graph_dict, node_accelerations)

    write_hdf5(data_dict, path=os.path.join(base_dir, "h5", "raw"))
    write_dgl(graph_dict, path=os.path.join(base_dir, "h5", "raw"))
    read_hdf5(path=os.path.join(base_dir, "h5", "raw"))
    read_dgl(path=os.path.join(base_dir, "h5", "raw"))
    logger.debug("Finished creating h5 from raw")
