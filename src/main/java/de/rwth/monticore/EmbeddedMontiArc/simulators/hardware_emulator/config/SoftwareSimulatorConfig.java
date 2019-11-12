/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.monticore.EmbeddedMontiArc.simulators.hardware_emulator.config;



public class SoftwareSimulatorConfig {

    String softwares_folder = ".";

    public String get_config_string() {
        return "softwares_folder=" + softwares_folder;
    }

    //The folder where the software program files are.
    public SoftwareSimulatorConfig set_softwares_folder(String path){
        this.softwares_folder = path;
        return this;
    }

}