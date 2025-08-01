import numpy as np
import glob, os
import gzip
import shutil
from pytz import NonExistentTimeError
from qd.cae.dyna import *
import pandas as pd
import h5py
import pickle 
import matplotlib.pyplot as plt
#from torch import _fake_quantize_learnable_per_tensor_affine
import mxnet as mx
import dgl
import ast 
import itertools
#import torch
from tqdm import tqdm
import plotly.graph_objects as go
from plotly.subplots import make_subplots


#TODO: 
#   - PCA
#   - 4000 Punkte bei den 4000 Punkten wirklich
#   - Parameter DoE Side Rail 13

def get_beam_width_array_side_rail(lattice_parameters):
    width = np.ones(52224)
    ct_block = 768
    for i in range(len(lattice_parameters)):
        width[i*ct_block:(i+1)*ct_block] = lattice_parameters[i] 
        width[(i+13)*ct_block:(i+13+1)*ct_block] = lattice_parameters[i] 
        width[(i+34)*ct_block:(i+34+1)*ct_block] = lattice_parameters[i]
        width[(i+47)*ct_block:(i+47+1)*ct_block] = lattice_parameters[i]
    
    for i in range(6):
        width[(i+26)*ct_block:(i+26+1)*ct_block] = lattice_parameters[i+7] 
        width[(i+60)*ct_block:(i+60+1)*ct_block] = lattice_parameters[i+7]
    
    for i in range(2):
        width[(i+32)*ct_block:(i+32+1)*ct_block] = lattice_parameters[i+11] 
        width[(i+66)*ct_block:(i+66+1)*ct_block] = lattice_parameters[i+11]

    return width

def get_crash_worthiness_indicators(displacement_force, fac=1):
    #https://sci-hub.se/10.1007/s12206-021-0226-8 gute Quelle fuer die Werte 
    energy = np.trapz(fac*displacement_force[1], x=displacement_force[0])
    crush_force_efficiency = np.max(fac*displacement_force[1])/np.mean(fac*displacement_force[1])
    return [energy, crush_force_efficiency]

def create_label(elem, crash_worthiness_indicators):
    label = str(elem[0]) + ": "
    if len(elem) > 3 and elem[3] >1:
        label += str(elem[3]) + "*"
    else: 
        label += ""
    for thickness in elem[1]:
        label += str(thickness) + "mm, "
    return label + "Energy absorbed: {:.2f}, CFE: {:.2f}".format(crash_worthiness_indicators[0], crash_worthiness_indicators[1])

def linear_model(fd_1, fd_2, delta_d=False):
    d_step = 150/len(fd_1[0])
    steps = int(150/d_step)
    f_all = np.zeros(steps)
    f1 = np.zeros(steps)
    f2 = np.zeros(steps)
    start = 0.0000001
    eps = 0.00000000000001
    d1_all = np.zeros(steps)
    d2_all = np.zeros(steps)
    d1_akt = start
    d2_akt = start
    d1_delta = 0
    d2_delta = 0
    for step in range(steps):
        deformed = False
        f1_akt = np.interp(d1_akt, fd_1[0], fd_1[1], right=0)
        f2_akt = np.interp(d2_akt, fd_2[0], fd_2[1], right=0)

        f1[step] = f1_akt
        f2[step] = f2_akt

        if d1_akt > np.max(fd_1[0]):
            deformed = True
            d1_delta = 0
            d2_delta = d_step
            k_ges = f2_akt/d2_delta
        else:
            d1_delta = (1-f1_akt/(f1_akt+f2_akt))*d_step
        if d2_akt > np.max(fd_2[0]):
            d1_delta = d_step
            d2_delta = 0
            k_ges = f1_akt/d1_delta
        elif not deformed:
            d2_delta = (1-f2_akt/(f1_akt+f2_akt))*d_step
            k1 = f1_akt/d1_delta
            k2 = f2_akt/d2_delta
            k_ges = (k1*k2)/(k1+k2)
        f_all[step] = k_ges*d_step
        d1_akt += d1_delta
        d2_akt += d2_delta
        d1_all[step] = d1_delta if delta_d else d1_akt
        d2_all[step] = d2_delta if delta_d else d2_akt
    return f_all, f1, f2, d1_all, d2_all

def get_stepwise_linear_model(data_dict, delta_d=False, to_approx="B_DoE", approx_with="A_DoE_100", f_all=False, test_plot=-1):
    num_exp = len(data_dict[to_approx].keys())
    td_data_number = data_dict[to_approx][str([1.,1.])].shape[1]*num_exp
    features = np.zeros((td_data_number, 4)) #f1, f2, d1, d2
    labels = np.zeros((td_data_number))
    ct = 0
    for i in np.arange(1.0, 5.1, 0.1):
        for j in np.arange(1.0, 5.1, 0.1):
                i = np.round(i, decimals=1)
                j = np.round(j, decimals=1)
                f_all,f1,f2,d1,d2 = linear_model(data_dict[approx_with][str([i])], data_dict[approx_with][str([j])], delta_d=delta_d)
                labels[ct*1000:(ct+1)*1000] = data_dict[to_approx][str([i,j])][1][:1000]
                features[ct*1000:(ct+1)*1000, 0] = f1
                features[ct*1000:(ct+1)*1000, 1] = f2
                features[ct*1000:(ct+1)*1000, 2] = d1
                features[ct*1000:(ct+1)*1000, 3] = d2
                if ct == test_plot:
                    plot_linear_model(f1, f2, d1, d2, f_all, data_dict[to_approx][str([i, j])], i, j, delta_d=delta_d)
                ct += 1
    return {"parameters" : features, "labels" : labels}


def unpack(folder, only_binout=False, delete=False):
    print("===== Unpack =====")
    for root, dirs, files in sorted(os.walk(folder)):
        for file in files:
            if file.endswith(".gz"):
                if not only_binout or "binout" in file:
                    file_path = os.path.join(root, file)
                    print("Unpacking:", file_path, end="-->")
                    with gzip.open(file_path, 'rb') as f_in:
                        with open(os.path.splitext(file_path)[0], 'wb') as f_out:
                            print(os.path.splitext(file_path)[0])
                            shutil.copyfileobj(f_in, f_out)
                            if delete:
                                os.remove(file_path)

def get_force_displacements(folder, end=-1, invalid_indices=None, two_binouts=True, two_d_disp=True, two_d_force=True, subfolder="", fd_start=0, fd_end=1000, first_keyword_force="rc_forc", force_dir='x_force', first_keyword_disp="nodout", disp_dir='x_displacement', zero_start=False, custom_disp=None, get_node_force=False, get_node_acceleration=False, get_node_velocity=False, t=range(0,200)):
    print("===== Getting FD =====")
    node_forces = []
    node_velocity = []
    node_acceleration = []
    fd = []
    start_t = min(t)
    end_t = max(t)
    ct = 0
    for file_or_folder in np.sort(os.listdir(folder)):
        complete_folder = os.path.join(folder, file_or_folder)  
        ct+=1 
        if os.path.isdir(complete_folder):
            folder_idx = int(complete_folder[-2:])-1
            if folder_idx == end-int(zero_start):
                break
            if invalid_indices == None or folder_idx not in invalid_indices:
                force = None
                displacement = None
                complete_file_1 = None
                complete_file_2 = None
                for file in os.listdir(os.path.join(complete_folder, subfolder)): 
                    complete_file = os.path.join(complete_folder, subfolder, file)  
                    if file.startswith("binout") and not file.endswith(".gz"):
                        binout = Binout(complete_file)
                        if get_node_force and 'nodfor' in binout.read():
                            node_force_sample = np.zeros((end_t-start_t, binout.read('nodfor', 'x_force').shape[1], 3))
                            node_force_sample[:, :, 0] = binout.read('nodfor', 'x_force')[start_t:end_t,:]
                            node_force_sample[:, :, 1] = binout.read('nodfor', 'y_force')[start_t:end_t,:]
                            node_force_sample[:, :, 2] = binout.read('nodfor', 'z_force')[start_t:end_t,:]
                            node_forces.append(node_force_sample)
                        if get_node_acceleration and 'nodout' in binout.read():
                            node_acceleration_sample = np.zeros((end_t-start_t, binout.read('nodout', 'x_acceleration').shape[1], 3))
                            node_acceleration_sample[:, :, 0] = binout.read('nodout', 'x_acceleration')[start_t:end_t,:]
                            node_acceleration_sample[:, :, 1] = binout.read('nodout', 'y_acceleration')[start_t:end_t,:]
                            node_acceleration_sample[:, :, 2] = binout.read('nodout', 'z_acceleration')[start_t:end_t,:]
                            node_acceleration.append(node_acceleration_sample)
                        if get_node_velocity and 'nodout' in binout.read():
                            node_velocity_sample = np.zeros((end_t-start_t, binout.read('nodout', 'x_velocity').shape[1], 3))
                            node_velocity_sample[:, :, 0] = binout.read('nodout', 'x_velocity')[start_t:end_t,:]
                            node_velocity_sample[:, :, 1] = binout.read('nodout', 'y_velocity')[start_t:end_t,:]
                            node_velocity_sample[:, :, 2] = binout.read('nodout', 'z_velocity')[start_t:end_t,:]
                            node_velocity.append(node_velocity_sample)
                    
                        if first_keyword_force in binout.read():
                            if two_d_force:
                                force = binout.read(first_keyword_force, force_dir)[fd_start:fd_end, 1]
                            else:
                                force = binout.read(first_keyword_force, force_dir)[fd_start:fd_end]
                            complete_file_1 = complete_file
                        if first_keyword_disp in binout.read() and not custom_disp is not None:
                            if two_d_disp:
                                displacement = np.abs(binout.read(first_keyword_disp, disp_dir)[fd_start:fd_end, 0])
                            else:
                                displacement = np.abs(binout.read(first_keyword_disp, disp_dir)[fd_start:fd_end])
                            
                            complete_file_2 = complete_file
                        else:
                            displacement = custom_disp
                if force is not None and displacement is not None:
                    print("FD:", complete_file_1, complete_file_2)
                    fd.append([displacement, force])
    return node_forces, node_velocity, node_acceleration, np.asarray(fd)

def add_feature(graph_dict, new_feature):
    i = 0
    for sample in graph_dict['node_features']:
        tmp = np.zeros((sample.shape[0], sample.shape[1], sample.shape[2]+new_feature[i].shape[2]))
        tmp[:, :, :-3] = sample
        tmp[:, :, -3:] = new_feature[i][:, :sample.shape[1], :]
        graph_dict['node_features'][i] = tmp
        i+=1

def get_d3plots(folder, end = -1, adjacency=True, adjacency_weighted=False, subfolder="", max_nodes= 103362, max_beams=52224, d3plot_range=range(0,40), lattice_parameters=None, different_shapes=False, zero_start=False, cube_gnn=False):
    print("===== Getting d3plots =====")
    data_dict = {"node_features" : []}
    if adjacency or adjacency_weighted:
        data_dict['adjacency'] = []
    invalid_indices = []
    if zero_start:
        cf_idx = -2
    else:
        cf_idx = -1
    for file_or_folder in np.sort(os.listdir(folder)):
        cf_idx+=1
        complete_folder = os.path.join(folder, file_or_folder)
        if os.path.isdir(complete_folder):
            folder_idx = int(complete_folder[-2:])-1
            if cf_idx < folder_idx:
                invalid_indices.append(cf_idx)
                cf_idx+=1
            if folder_idx == end-int(zero_start):
                break
            if invalid_indices==None or folder_idx not in invalid_indices:
                if not os.path.isfile(os.path.join(complete_folder, subfolder, "d3plot")):
                    invalid_indices.append(folder_idx)
                    continue
                d3plot = D3plot(os.path.join(complete_folder, subfolder, "d3plot"))
                print("D3:", os.path.join(complete_folder, subfolder, "d3plot"))
                num_timesteps = max(d3plot_range)-min(d3plot_range) #d3plot.get_nTimesteps(), 25 is the minimal number of timesteps
                if cube_gnn:
                    max_nodes = d3plot.get_nNodes()
                node_coords_over_time = np.zeros((num_timesteps, max_nodes, 3)) 
                d3plot.read_states('disp')
                
                i = 0
                if d3plot.get_nodes()[0].get_coords().shape[0] < max(d3plot_range)+1:
                    invalid_indices.append(folder_idx)
                    continue
                for node in d3plot.get_nodes()[:max_nodes]:
                    node_coords_over_time[:num_timesteps, i, :] = node.get_coords()[min(d3plot_range):max(d3plot_range), :max_nodes]
                    i += 1
                beams = d3plot.get_element_node_ids(Element.beam, 2)[:max_beams, :]
                mapping = dict(zip(d3plot.get_node_ids(), np.arange(len(d3plot.get_node_ids()))))
                beams = np.vectorize(mapping.get)(beams)
                unqiue_node_ids = np.unique(beams.flatten())
                new_node_coords_over_time = node_coords_over_time[:, unqiue_node_ids, :]
                data_dict['node_features'].append(new_node_coords_over_time)
                mapping = dict(zip(unqiue_node_ids, np.arange(len(unqiue_node_ids))))
                beams = np.vectorize(mapping.get)(beams)
                if adjacency_weighted:
                    width = get_beam_width_array_side_rail(lattice_parameters)
                    beams_weighted = np.zeros((len(beams), 3))
                    beams_weighted[:, :2] = beams
                    beams_weighted[:, 2] = width
                    data_dict["adjacency"] = beams_weighted
                    continue
                if adjacency:
                    data_dict["adjacency"].append(beams)
    if different_shapes:
        max_nodes = max([elem.shape[1] for elem in data_dict["node_features"]])
        nodes = np.zeros((len(data_dict["node_features"]), max(d3plot_range), max_nodes, 3))
        idx = 0
        for elem in data_dict["node_features"]:
            nodes[idx, :, :elem.shape[1], :]=elem 
            idx+=1
        data_dict["node_features"] = nodes 
    #to_numpy(dict)
    return data_dict, invalid_indices

def to_numpy(dict):
    for key in dict.keys():
        dict[key] = np.asarray(dict[key])
    return dict

def csv_based_routine(folder, end=-1):
    edge_index_files = os.path.join(folder, 'f2ccz_r{}um_a{}mm_report_ind_mod.csv')
    node_files = os.path.join(folder,'f2ccz_r{}um_a{}mm_report_nodes_mod.csv')
    edge_files = os.path.join(folder,'f2ccz_r{}um_a{}mm_report_elements_mod.csv')
    result_files = os.path.join(folder,'f2ccz_r{}um_a{}mm_report_FU.txt')
    file_name_templates = [node_files, edge_index_files, edge_files, result_files]

    edge_radius = np.linspace(500, 1500, 11, dtype=int)
    edge_length = np.linspace(15, 25, 11, dtype=int)

    map_dataframe = lambda x: ast.literal_eval(x)

    inner_edge_vertex = True

    files = [name.format(radius, length) for radius, length, name in itertools.product(edge_radius, edge_length, file_name_templates)]
    grouped_files = [files[x:x + 4] for x in range(0, len(files), 4)]
    adjacency = np.zeros((len(grouped_files), 1480, 2))
    edge_weight = np.zeros((len(grouped_files), 200, 1480))
    features = np.zeros((len(grouped_files), 200, 1025, 9))
    predictions_label = np.zeros((len(grouped_files), 200, 2))

    ct = 0
    for index, sample in tqdm(enumerate(grouped_files), bar_format='{l_bar}{bar:60}{r_bar}{bar:-10b}'):
        node_data = pd.read_csv(sample[0], header=None)
        edge_index_data = pd.read_csv(sample[1], header=None)
        edge_data = pd.read_csv(sample[2], header=None)
        with open(sample[3]) as f:
            result_data = ast.literal_eval(f.readline())
        if not inner_edge_vertex:
            x = torch.stack([torch.cat([torch.Tensor(node_data.iloc[t, :285].map(map_dataframe)),
                                        torch.Tensor(edge_data.iloc[t].map(map_dataframe))[:285, :6]], dim=1) for t
                            in range(len(node_data))])
            edge_attr = torch.unsqueeze(torch.Tensor(edge_data.iloc[0].map(map_dataframe))[:, -1], 0).repeat(200, 1)
            edge_index = torch.tensor(edge_index_data.iloc[0:2].to_numpy(), dtype=torch.int64)
        else:
            main_node_attrs = torch.stack(
                [torch.Tensor(edge_data.iloc[t].map(map_dataframe))[:285, :6] for t in range(len(node_data))])
            inner_node_attrs = torch.stack(
                [torch.Tensor(edge_data.iloc[t].map(map_dataframe))[:, 12:18] for t in range(len(node_data))])
            all_node_attrs = torch.cat([main_node_attrs, inner_node_attrs], dim=1)
            node_coordinates = torch.stack(
                [torch.Tensor(node_data.iloc[t].map(map_dataframe)) for t in range(len(node_data))])
            x = torch.cat([node_coordinates, all_node_attrs], dim=-1)
            edge_attr = torch.unsqueeze(torch.Tensor(edge_data.iloc[0].map(map_dataframe))[:, -1], 0).repeat(200, 2)
            edge_index = torch.cat((torch.tensor(edge_index_data.iloc[0:3:2].to_numpy(), dtype=torch.int64),
                                        torch.tensor(edge_index_data.iloc[1:3].to_numpy(), dtype=torch.int64)), dim=1)
            y = torch.Tensor(result_data[1::5])  # Results have 1000 time steps in sample dataset

        adjacency[index] = edge_index.T
        edge_weight[index] = edge_attr
        features[index] = x
        predictions_label[index] = y
        ct += 1
        if ct == end:
            break
    return {"fd_label":predictions_label} , {"adjacency" : adjacency.astype(np.int32), "node_features":features}

def collapse_preprocess(doe_list):
    num_doe = sum([elem.shape[0] for elem in doe_list])
    features = np.zeros((num_doe, 13))
    labels = np.zeros(num_doe)
    start = 0
    for elem in doe_list:
        features[start:start+elem.shape[0], :] = elem[:, 1:14]
        labels[start:start+elem.shape[0]] = elem[:, -1]
        start += elem.shape[0]
    return {"parameters":features, "collapse_label" : labels}


def read_params(doe_matrix_file, invalid_indices=None, end=-1, excel=False, header=None):
    print("===== Reading Params =====")
    if excel:
        df = pd.read_excel(doe_matrix_file)
        del df['M']
        del df['Algorithm']
        del df['Phase']
        del df['E_Int']
        del df['E_Int_mass']
        del df['MaxForce']
        del df['mass']
        if end > -1:
            df = df.head(end)
        if invalid_indices is not None:
            df = df.drop(invalid_indices)
    else:
        df = pd.read_csv(doe_matrix_file, header=header)
        if end > -1:
            df = df.head(end)
        if invalid_indices is not None:
            df = df.drop(invalid_indices)
    print("Reading params done!\n")
    return df.to_numpy(dtype=np.float)

def read_params_nested(thickness_file, ref, invalid_indices=None, end=-1, excel=False, header=None):
    print("===== Reading Params =====")
    df_all = pd.read_excel(thickness_file, engine='openpyxl', header=None).to_numpy()
    df_ref = pd.read_excel(ref, engine='openpyxl', header=0).to_numpy()
    #print(df_all)
    #print(df_ref)
    res = np.zeros((5125, 6))
    res[:, 0] = df_all[:, 1]
    res[:, 1:] = df_ref[df_all[:,0].astype(int)-1, 1:6]
    print("Reading params done!\n")
    if end < 0:
        return res
    else:
        return res[:end]
    

def plot_data_dict(data_dict, list, sum_energies=True):
    types = {"A_DoE": "-","B_DoE": "-","C_DoE": "-","D_DoE": ":","F_DoE": "-.", "G_DoE": ":"}
    energies = []
    for elem in list:
        string = str(elem[1])
        displacement_force = data_dict[elem[0]][string]
        offset = 0
        fac = 1
        if len(elem) >= 3:
            offset += elem[2]
        if len(elem) >= 4:
            fac*=elem[3]
        if len(elem) >= 5:
            if elem[4] < 0:
                displacement_force[0][-displacement_force[1]<-elem[4]] = 0
            else:
                displacement_force[0][-displacement_force[1]>elem[4]] = 0

        energy, crush_force_efficiency = get_crash_worthiness_indicators(displacement_force, fac)
        energies.append(energy)
        plt.plot(displacement_force[0]+offset, displacement_force[1]*fac,types[elem[0]], label=create_label(elem, (energy, crush_force_efficiency)))
    if sum_energies:
        energies = np.asarray(energies)
        energy_string = "Sum of absorbed energies {:.2f}, {:.2f}% of energy absorbed by compound".format( np.sum(energies[:-1]), 100*(np.sum(energies[:-1]))/energies[-1])
        plt.plot([0], [0], label=energy_string)
    plt.legend()
    plt.xlabel("Time (displacement)")
    plt.ylabel("Force [kN]")
    plt.show()

def plot_linear_model(f1, f2, d1, d2, f_all, label, first_block, second_block, delta_d=False):
    plt.title("Linear model for {} and {}".format(first_block, second_block))
    if delta_d:
        plt.plot([np.sum(d1[:i]) for i in range(len(d1))], f1, label="Force of Subblock 1")
        plt.plot([np.sum(d2[:i]) for i in range(len(d2))], f2, label="Force of Subblock 2")
    else:
        plt.plot(d1, f1, label="Force of Subblock 1")
        plt.plot(d2, f2, label="Force of Subblock 1")
            
    plt.plot(label[0], label[1], label="F label")
    plt.plot(label[0], f_all, label="F linear Model")
    plt.xlabel("Time (displacement)")
    plt.ylabel("Force [kN]")
    plt.legend()
    plt.show()


def write_hdf5(all_data, path="./h5/raw/", shuffle = False, train_test_split=1, train_file="train.h5", test_file="test.h5", prefix=[]):
    if shuffle:
        shuffle = np.arange(all_data[list(all_data.keys())[0]].shape[0])
        np.random.shuffle(shuffle)
        for key in all_data.keys():
            all_data[key] = all_data[key][shuffle]
    end_idx = int(all_data[list(all_data.keys())[0]].shape[0]*train_test_split)
    
    for pre in prefix:
        train_file = pre + "_" + train_file
        #test_file = pre + "_" + test_file

    train_file = os.path.join(path, train_file)
    train = h5py.File(train_file, "w")
    for key in all_data.keys():
            train.create_dataset(key, data = all_data[key][:end_idx].astype(np.float32))
    
    test_file = os.path.join(path, test_file)
    test = h5py.File(test_file, "w")
    for key in all_data.keys():
            test.create_dataset(key, data = all_data[key][end_idx:].astype(np.float32))

def read_hdf5(path="./h5/raw/", train_file="train.h5", test_file="test.h5",prefix=[]):
    for pre in prefix:
        train_file = pre + "_" + train_file
        #test_file = pre + "_" + test_file
    train_file = os.path.join(path, train_file)
    #test_file = os.path.join(path, test_file)
    train = h5py.File(train_file, 'r')
    #test = h5py.File(test_file, 'r')
    print("\n========== Train ==========")
    print("File:", train_file)
    for key in train.keys():
        print(key, train[key].shape)
    #print("\n========== Test ===========")
    #print("File:", test_file)
    #for key in test.keys():
    #    print(key, test[key].shape)
    return train#, test

def read_dgl(path="./h5/raw/", train_file="train_graph", test_file="test_graph",prefix=[]):
    for pre in prefix:
        train_file = pre + "_" + train_file
        #test_file = pre + "_" + test_file
    train_file = os.path.join(path, train_file)
    #test_file = os.path.join(path, test_file)
    train_graph = dgl.load_graphs(train_file)
    #test_graph = dgl.load_graphs(test_file)
    print("\n==========DGL Train==========")
    print("File:", train_file)
    print(train_graph)
    #print("\n==========DGL Test==========")
    #print("File:", test_file)
    #print(test_graph)
    return train_graph#, test_graph

def write_dgl(graph_data, path="./h5/raw/", train_file="train_graph", test_file="test_graph", train_test_split=1):
    print("===== DGL Graph Write =====")
    adjacency = graph_data['adjacency']
    features = graph_data['node_features']
    sample_number = 0
    graph_list = []
    for sample in adjacency:
        graph_sample = dgl.graph((sample[:,0], sample[:, 1]))
        graph_sample = dgl.to_bidirected(graph_sample)
        graph_list.append(dgl.add_self_loop(graph_sample))
        feature_sample = np.swapaxes(features[sample_number], 0, 1)
        feature_sample = mx.nd.array(feature_sample)
        feature_sample = feature_sample[0:graph_list[sample_number].num_nodes()]
        feature_sample = mx.nd.flatten(feature_sample)
        graph_list[sample_number].ndata['features_'] = feature_sample
        sample_number += 1
    #end_idx = int(len(graph_list)*train_test_split)
    dgl.save_graphs(os.path.join(path, train_file), graph_list)
    #dgl.save_graphs(os.path.join(path, test_file), graph_list[end_idx:])
    return

start_frame_idx = 0
end_frame_idx = 7

def plot_geometry(x, y, z,  x_line, y_line, z_line, file_name="plot"):
    fig = go.Figure(data=[go.Scatter3d(
        x=x[start_frame_idx],
        y=y[start_frame_idx],
        z=z[start_frame_idx],
        mode='markers',
        marker=dict(size=3),
        name="nodes"),
        go.Scatter3d(
            x=x_line[start_frame_idx],
            y=y_line[start_frame_idx],
            z=z_line[start_frame_idx],
            mode='lines',
            line=dict(width=3),
            name='struts'
        )],)
    eps = 10
    fig.update_layout(
        scene=dict(
            xaxis=dict(nticks=4, range=[np.min(x)-eps, np.max(x)+eps], ),
            yaxis=dict(nticks=4, range=[np.min(y)-eps, np.max(y)+eps], ),
            zaxis=dict(nticks=4, range=[np.min(z)-eps, np.max(z)+eps], ), ),
        updatemenus=[dict(type="buttons", buttons=[
        dict(label="Play",
             method="animate",
             args=[None, dict(frame=dict(redraw=True, fromcurrent=True, mode='immediate'))]),
        dict(label="Pause",
             method="animate",
             args=[None, dict(frame=dict(duration=0, redraw=False), mode='immediate', transition=dict(duration=0))])
        ])],
        # title="Cell type: " + cell_type + ", Radius: " + str(radius) + "µm, strut thickness: " + str(width) + "mm"
    )
    frames = [go.Frame(data=[
        go.Scatter3d(
            x=x[k],
            y=y[k],
            z=z[k],
            mode='markers',
            marker=dict(size=3),
            name="nodes"),
        go.Scatter3d(
            x=x_line[k],
            y=y_line[k],
            z=z_line[k],
            mode='lines',
            line=dict(width=3),
            name='struts'
        )],
        name=f'frame{k}'
    ) for k in range(start_frame_idx, end_frame_idx)]
    fig.layout.scene.camera.projection.type = "orthographic"
    fig.update(frames=frames)
    fig.layout.updatemenus[0].buttons[0].args[1]['frame']['duration'] = 30
    fig.write_html(file_name + ".html")


def get_force_displacement_vectors(sample):
    displacement = sample.y[:, 0, 0].numpy()
    force = sample.y[:, 0, 1].numpy()
    return displacement, force


def get_coordinate_vectors(node, ad):
    x = node[:, :, 0]
    y = node[:, :, 1]
    z = node[:, :, 2]
    edge_indice = ad
    x_lines = []
    y_lines = []
    z_lines = []
    for t in range(x.shape[0]):
        x_lines_t = []
        y_lines_t = []
        z_lines_t = []
        for i, ind in enumerate(edge_indice):
            x_lines_t.extend([x[t, ind[0]], x[t, ind[1]], np.nan])
            y_lines_t.extend([y[t, ind[0]], y[t, ind[1]], np.nan])
            z_lines_t.extend([z[t, ind[0]], z[t, ind[1]], np.nan])
        x_lines.append(x_lines_t)
        y_lines.append(y_lines_t)
        z_lines.append(z_lines_t)

    return x, y, z, np.asarray(x_lines), np.asarray(y_lines), np.asarray(z_lines)


def create_sample_plot(node, ad, file_name="plot"):
    x, y, z, x_line, y_line, z_line = get_coordinate_vectors(node, ad)
    plot_geometry(x, y, z, x_line, y_line, z_line, file_name)
