/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.dynamicinterface;

import java.util.Vector;

import de.rwth.montisim.commons.dynamicinterface.PortInformation.PortDirection;
import de.rwth.montisim.commons.dynamicinterface.PortInformation.PortType;

public class ProgramInterface {
    public static final String CURRENT_VERSION = "2.0";
    public String name;
    // The version should correspond to 'CURRENT_VERSION': allows to detect programs using an old version of the interface.
    public String version; 
    public Vector<PortInformation> ports = new Vector<>();

    @Override
    public String toString() {
        String res = "ProgramInterface of "+name+ " (v" + version+"):\n";
        for (PortInformation p : ports) {
            res += '\t';
            if (p.port_type == PortType.SOCKET) res += "socket ";
            if (p.direction == PortDirection.INPUT) res += "I  ";
            else if (p.direction == PortDirection.OUTPUT) res += "O  ";
            else res += "IO ";
            res += p.name + ": "+p.data_type.toString()+"\n";
        }
        return res;
    }

    public boolean isVersionValid() {
        return version.equals(CURRENT_VERSION);
    }
}
