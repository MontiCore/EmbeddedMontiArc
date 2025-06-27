/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.testing;

import java.util.ArrayList;
import java.util.List;

/**
 */
public class StreamTestPort {
    protected String name;
    protected List<StreamTestValue> values = new ArrayList<>();

    public StreamTestPort() {
    }

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

    public List<StreamTestValue> getValues() {
        return values;
    }

    public void setValues(List<StreamTestValue> values) {
        this.values = values;
    }

    public void addValue(StreamTestValue value) {
        this.values.add(value);
    }
}
