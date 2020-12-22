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
            if (p.direction == PortDirection.INPUT) res += (p.port_type == PortType.SOCKET) ? "sock_in" : "I ";
            else res += (p.port_type == PortType.SOCKET) ? "sock_out" : "O ";
            res += p.name + ": "+p.data_type.toString()+"\n";
        }
        return res;
    }

    public boolean isVersionValid() {
        return version.equals(CURRENT_VERSION);
    }
}
