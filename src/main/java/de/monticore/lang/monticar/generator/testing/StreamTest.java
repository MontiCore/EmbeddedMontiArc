/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.monticore.lang.monticar.generator.testing;

import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;

/**
 * @author Sascha Schneiders
 */
public class StreamTest {
    protected String name;
    protected String packageName;
    protected String componentName;
    protected List<StreamTestPort> ports = new ArrayList<>();

    public StreamTest() {
    }

    public List<StreamTestPort> getPorts() {
        return ports;
    }

    public void setPorts(List<StreamTestPort> ports) {
        this.ports = ports;
    }

    public void addPort(StreamTestPort port) {
        this.ports.add(port);
    }

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

    public String getPackageName() {
        return packageName;
    }

    public void setPackageName(String packageName) {
        this.packageName = packageName;
    }

    public String getComponentName() {
        return componentName;
    }

    public void setComponentName(String componentName) {
        this.componentName = componentName;
    }

    @Override
    public String toString() {
        StringBuilder result = new StringBuilder();
        result.append("package ");
        result.append(packageName);
        result.append(";\n");
        result.append("stream ");
        result.append(name);
        result.append(" for ");
        result.append(componentName);
        result.append("{\n");

        for (StreamTestPort port : ports) {
            result.append(port.getName());
            result.append(": ");
            for (int i = 0; i < port.getValues().size(); ++i) {
                StreamTestValue value = port.getValues().get(i);
                result.append(value.getStringRepresentation());
                result.append(" ");
                if (i + 1 < port.getValues().size())
                    result.append("tick ");
            }
            result.append(";\n");
        }

        result.append("}");
        return result.toString();
    }

    public void fixPortArrays() {
        boolean notExit = true;
        while (notExit) {
            notExit = false;
            for (StreamTestPort matchPort : ports) {
                List<StreamTestPort> samePorts = getAllArrayPortsWithSameName(matchPort);
                if (samePorts.size() > 0) {
                    notExit = true;
                    ports.removeAll(samePorts);
                    ports.add(createNewStreamTestPort(samePorts));
                    break;
                }
            }
        }
    }

    public StreamTestPort createNewStreamTestPort(List<StreamTestPort> samePorts) {
        StreamTestPort port = new StreamTestPort();
        port.setName(getNameWithoutArrayPart(samePorts.get(0).getName()));
        for (int i = 0; i < samePorts.get(0).getValues().size(); ++i) {
            StringBuilder valueString = new StringBuilder();
            valueString.append("[");
            for (int j = 0; j < samePorts.size(); ++j) {
                StreamTestPort port2 = samePorts.get(j);
                valueString.append(port2.getValues().get(i).getStringRepresentation());
                if (j + 1 < samePorts.size()) {
                    valueString.append(", ");
                }
            }
            valueString.append("]");
            port.addValue(new BasicStreamTestValue<String>(valueString.toString()));
        }
        return port;
    }

    /**
     * Includes the provided port
     *
     * @param matchPort
     * @return
     */
    public List<StreamTestPort> getAllArrayPortsWithSameName(StreamTestPort matchPort) {
        List<StreamTestPort> samePorts = new ArrayList<>();

        String matchName = getNameWithoutArrayPart(matchPort.getName());
        for (StreamTestPort port : ports) {
            if (matchPort != port) {
                String curName = getNameWithoutArrayPart(port.getName());
                if (curName.equals(matchName)) {
                    Log.debug(matchPort.getName() + " " + port.getName() + " for names " + matchName + " " + curName, "Found Same Ports");
                    if (!samePorts.contains(port))
                        samePorts.add(port);
                    if (!samePorts.contains(matchPort)) {
                        samePorts.add(matchPort);
                    }
                }
            }
        }
        return samePorts;
    }

    public static String getNameWithoutArrayPart(String name) {
        int index = name.indexOf("[");
        if (index == -1)
            index = name.length();
        return name.substring(0, index);
    }
}
