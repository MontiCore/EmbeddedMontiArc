/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.testing;

import java.util.ArrayList;
import java.util.List;

/**
 */
public class PortNameValues {
    String portName;
    List<String> values = new ArrayList<>();

    public PortNameValues() {
    }

    public PortNameValues(String portName, String firstValue) {
        this.portName = portName;
        this.values.add(firstValue);
    }

    public PortNameValues(String portName, List<String> values) {
        this.portName = portName;
        this.values = values;
    }

    public String getPortName() {
        return portName;
    }

    public void setPortName(String portName) {
        this.portName = portName;
    }

    public List<String> getValues() {
        return values;
    }

    public void setValues(List<String> values) {
        this.values = values;
    }

    public void addValue(String value) {
        this.values.add(value);
    }
}
