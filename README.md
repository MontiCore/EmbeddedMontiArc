# LTSAutopilot
LTSAutopilot is applied for the decisions in cooperative driving scenarios. Current project is supported with Smartfoxserver 2X, which provides JavaScript API for the data exchange of visualization.

# Installation of SmartFoxServer 2X
Running the complete simulation platform with current LTSAutopilot requires deployment of the visualization and server repository deliverables to an installed SmartFoxServer 2X.<br>
The [Installision of SmartFoxServer 2X](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/server?nav_source=navbar) and the [Configuration of Database Service](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/server?nav_source=navbar) have been already introduced in server project. 


# Generation of AutopilotAdapter
LTSAutopilot Model in the folder _model_ contains the core algorithm(protocol) of cooperative driving, it firstly need to be generated to _c++_ file, and then compiled to AutopilotAdapter.dll. Current AutopilotAdapter.dll file is in the folder _dll_.<br>
The process of generation is in the Project [EMAM-showcase](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/EMAM-showcase?nav_source=navbar). When the model is generated, the emam2cpp.jar in [EMAM-showcase](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/EMAM-showcase?nav_source=navbar) will be replaced by the new emam2cpp.jar, which is in root folder and genedated from [EMAM2Cpp](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/EMAM2Cpp?nav_source=navbar) project (Weixu Branch).  
# Deploy of Autopilotadapter in RMIMODELServer
Once AutopilotAdapter.dll is generated, it will be deployed for the visualization of simulation in [RMIMODELServer](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/RMIModelServer/tree/Weixu)(Weixu Branch).
# Running scenario 
Several scenarios are listed in the folder _scenario examples_ , the simulation file and map file should be uploaded at the same time. After that the simulation can be started, and the scenario will be visualized.
 _Aachen.osm_ is the common map for these scenarios, it's defined in the area near Forckenbeckstraße and Pauwelsstraße in Aachen.
# Possible improvement of model language in future

