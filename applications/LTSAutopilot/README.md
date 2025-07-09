<!-- (c) https://github.com/MontiCore/monticore -->
# LTSAutopilot
LTSAutopilot is applied for the decisions in cooperative driving scenarios. Current project is supported with Smartfoxserver 2X, which provides JavaScript API for the data exchange of visualization.

## Installation of SmartFoxServer 2X
Running the complete simulation platform with current LTSAutopilot requires deployment of the visualization and server repository deliverables to an installed SmartFoxServer 2X.<br>
The [Installision of SmartFoxServer 2X](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/server?nav_source=navbar) and the [Configuration of Database Service](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/server?nav_source=navbar) have been already introduced in server project. 


## Generation of AutopilotAdapter
LTSAutopilot Model in the folder _model_ contains the core algorithm(protocol) of cooperative driving, it firstly need to be generated to _c++_ file, and then compiled to AutopilotAdapter.dll. Current AutopilotAdapter.dll file is in the folder _dll_.<br>
The process of generation is in the Project [EMAM-showcase](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/EMAM-showcase?nav_source=navbar). When you generate the model, please replace the emam2cpp.jar in [EMAM-showcase](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/EMAM-showcase?nav_source=navbar) by emam2cpp.jar, which is in root folder of this repository and genedated from [EMAM2Cpp](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/EMAM2Cpp/tree/Weixu) project (Weixu Branch).  
## Deploy of Autopilotadapter in RMIMODELServer
Once AutopilotAdapter.dll is generated, it will be deployed for the visualization of simulation in [RMIMODELServer](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/RMIModelServer/tree/Weixu)(Weixu Branch).
## Running scenario 
Several scenarios are listed in the folder _scenario examples_ , the simulation file and map file should be uploaded in the visualization part at the same time. After that the simulation can be started, and the scenario will be visualized.
 _Aachen.osm_ is the common map for these scenarios, it's defined in the area near Forckenbeckstraße and Pauwelsstraße in Aachen.
## Possible improvement of model language in future

   * #### More functions in new syntax for development <br>
     During the future development of the new syntax, it's better that more functions in syntax can be developed.<br>
     For a simple “loop” example, showing as follows:<br>
     ```js
        for i = 1:32
            if is_connected(plat_msg,i)
                for j = 1:list_num
                    if pmsg.vehicleid == platoonlist(j)
                        for k = 1:32
                            if is_connected(traj_msg,k)
                               tmsg = traj_msg(k);
                               if pmsg.vehicleid == tmsg.vehicleid
                                 ...
                               end
                            end
                        end
                    end
                end
            end
        end
     ```
     This part aims to select the platoon message and corresponding trajectory message according to the platoon id lists, but there are too many loop functions, resulting in kind of hard to read.<br>
     The new syntax with new function may like this:
     ```js
        for j = 1:list_num
            pmsg = select(platoonlist(j), plat_msg, id)
            tmsg = select(platoonlist(j), traj_msg, id)
            if pmsg.id != 0 && tmsg.id != 0 
               ...
            end 
        end
     ```
     With new function `select`, The High frequency of loop function can be reduced, and the readability of model is increased accordingly.

   * #### More error information in console
     More error information should be showed while the model is generated to cpp files, It's helpful to locate the precise error place. 
   
   * #### Usage of IDE 
     With the using of IDE, it can save works while developing.
  

