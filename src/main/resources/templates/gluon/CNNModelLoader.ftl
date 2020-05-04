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
    std::vector<Symbol> network_symbol_list;
    std::vector<std::map<std::string, NDArray>> network_param_map_list;
    
    std::vector<Symbol> loss_symbol;
    std::vector<std::map<std::string, NDArray>> loss_param_map;
    
    std::vector<Symbol> querry_symbol_list;
    std::vector<std::map<std::string, NDArray>> querry_param_map_list;


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

    void loadComponent(std::string json_path, 
                       std::string param_path,
                       std::vector<Symbol> &symbols_list, 
                       std::vector<std::map<std::string, NDArray>> &param_map_list){
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
    explicit ModelLoader(std::string file_prefix, std::vector<std::string> replay_querry_prefixes, mx_uint num_subnets, Context ctx_param){

        ctx = ctx_param;
        std::string network_json_path;
        std::string network_param_path;
        std::string loss_json_path;
        std::string loss_param_path;
        std::string querry_json_path;
        std::string querry_param_path;
        
        //assert(num_subnets-1 == replay_querry_prefixes.size());
        
        //Load network
        if(!num_subnets){
            network_json_path = file_prefix + "-symbol.json";
            network_param_path = file_prefix + "-0000.params";
            loadComponent(network_json_path, network_param_path, network_symbol_list, network_param_map_list);
        }else{
            for(int i=0; i < num_subnets; i++){
                network_json_path = file_prefix + "_sub_net_" + std::to_string(i) + "-symbol.json";
                network_param_path = file_prefix + "_sub_net_" + std::to_string(i) + "-0000.params";
                loadComponent(network_json_path, network_param_path, network_symbol_list, network_param_map_list);
                
                if(i >= 1 && !replay_querry_prefixes.empty()){
                    querry_json_path = replay_querry_prefixes[i] + "json";
                    querry_param_path = replay_querry_prefixes[i] + "params";
                    loadComponent(querry_json_path, querry_param_path, querry_symbol_list, querry_param_map_list);
                }
            }
        }

        //Load Loss
        loss_json_path = file_prefix + "_loss-symbol.json";
        loss_param_path = file_prefix + "_loss-0000.params";
        loadComponent(loss_json_path, loss_param_path, loss_symbol, loss_param_map);

        NDArray::WaitAll();
    }

    std::vector<Symbol> GetNetworkSymbols() {
        return network_symbol_list;
    }

    std::vector<std::map<std::string, NDArray>> GetNetworkParamMaps() {
        return network_param_map_list;
    }

    Symbol GetLoss() {
        return loss_symbol[0];
    }

    std::map<std::string, NDArray> GetLossParamMap() {
        return loss_param_map[0];
    }
    
   std::vector<Symbol> GetQuerrySymbols() {
        return querry_symbol_list;
    }

    std::vector<std::map<std::string, NDArray>> GetQuerryParamMaps() {
        return querry_param_map_list;
    }
};

#endif // CNNBUFFERFILE_H