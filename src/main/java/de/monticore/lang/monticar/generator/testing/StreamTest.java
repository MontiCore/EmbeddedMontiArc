package de.monticore.lang.monticar.generator.testing;

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
}
