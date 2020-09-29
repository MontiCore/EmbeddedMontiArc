/**
 * (c) https://github.com/MontiCore/monticore
 */
package de.rwth.montisim.hardware_emulator.config;

import java.io.Serializable;


public class SoftwareSimulatorConfig implements Serializable {

    private static final long serialVersionUID = 60388757112555139L;
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