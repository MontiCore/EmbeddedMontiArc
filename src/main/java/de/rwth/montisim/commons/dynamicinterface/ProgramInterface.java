package de.rwth.montisim.commons.dynamicinterface;

import java.util.Vector;

import de.rwth.montisim.commons.dynamicinterface.PortInformation.PortDirection;

public class ProgramInterface {
    public String name;
    public String version;
    public Vector<PortInformation> ports = new Vector<>();

    @Override
    public String toString() {
        String res = "ProgramInterface of "+name+ " (v" + version+"):\n";
        for (PortInformation p : ports) {
            res += '\t';
            if (p.direction == PortDirection.INPUT) res += "I ";
            else res += "O ";
            res += p.name + ": "+p.type.toString()+"\n";
        }
        return res;
    }
}
