#ifndef CNNBUFFERFILE_H
#define CNNBUFFERFILE_H

#include <mxnet-cpp/MxNetCpp.h>

#include <stdio.h>
#include <iostream>
#include <fstream>

using namespace mxnet::cpp;

// Read files to load moddel symbol and parameters
class ModelLoader {
 private :
    Context ctx = Context::cpu();
    std::vector<Symbol> symbols_list;
    std::vector<std::map<std::string, NDArray>> param_map_list;

    void checkFile(std::string file_path){
        std::ifstream ifs(file_path.c_str(), std::ios::in | std::ios::binary);
        if (!ifs) {
            std::cerr << "Can't open the file. Please check " << file_path << ". \n";
            return;
        }

        int length_;
        ifs.seekg(0, std::ios::end);
        length_ = ifs.tellg();
        ifs.seekg(0, std::ios::beg);
        std::cout << file_path.c_str() << " ... "<< length_ << " bytes\n";
        ifs.close();
    }

    void loadComponent(std::string json_path, std::string param_path){
        checkFile(json_path);
        symbols_list.push_back(Symbol::Load(json_path));
        checkFile(param_path);
        std::map<std::string, NDArray> params;
        NDArray::Load(param_path, 0, &params);
        param_map_list.push_back(processParamMap(params));
    }

    std::map<std::string, NDArray> processParamMap(std::map<std::string, NDArray> param_map){
        std::map<std::string, NDArray> processed_param_map;
        if(!param_map.empty()){
            for (const auto &pair : param_map) {
                std::string name = pair.first.substr(4); //the first four letters would be the type (arg: or aux:, but we don't have aux parameters? <- need to make sure)
                processed_param_map[name] = pair.second.Copy(ctx);
            }
        }
        return processed_param_map;
    }

 public :
    explicit ModelLoader(std::string file_prefix, mx_uint num_subnets, Context ctx_param){

        ctx = ctx_param;
        std::string full_json_path;
        std::string full_param_path;

        //Load network
        if(!num_subnets){
            full_json_path = file_prefix + "-symbol.json";
            full_param_path = file_prefix + "-0000.params";
            loadComponent(full_json_path, full_param_path);
        }else{
            for(int i=0; i < num_subnets; i++){
                full_json_path = file_prefix + "_sub_net_" + std::to_string(i) + "-symbol.json";
                full_param_path = file_prefix + "_sub_net_" + std::to_string(i) + "-0000.params";
                loadComponent(full_json_path, full_param_path);
            }
        }

        //Load Loss
        full_json_path = file_prefix + "_loss-symbol.json";
        full_param_path = file_prefix + "_loss-0000.params";
        loadComponent(full_json_path, full_param_path);

        NDArray::WaitAll();
    }

    std::vector<Symbol> GetNetworkSymbols() {
        std::vector<Symbol> network_symbols = symbols_list;
        network_symbols.pop_back();
        return network_symbols;
    }

    std::vector<std::map<std::string, NDArray>> GetNetworkParamMaps() {
        std::vector<std::map<std::string, NDArray>> param_maps = param_map_list;
        param_maps.pop_back();
        return param_maps;
    }

    Symbol GetLoss() {
        return symbols_list.back();
    }

    std::map<std::string, NDArray> GetLossParamMap() {
        return param_map_list.back();
    }
};

#endif // CNNBUFFERFILE_H