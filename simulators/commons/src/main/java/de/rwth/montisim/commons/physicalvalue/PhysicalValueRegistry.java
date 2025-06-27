/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.physicalvalue;

import java.util.HashMap;

import de.rwth.montisim.commons.utils.BuildObject;

public class PhysicalValueRegistry implements BuildObject {
    public static final String CONTEXT_KEY = "physical_values";
    HashMap<String, PhysicalValue> physicalValues = new HashMap<>();

    /** Should only be for RESOLVING the physical values, not repeated access. */
    public PhysicalValue getPhysicalValue(String name) {
        PhysicalValue res = physicalValues.get(name);
        if (res == null) throw new IllegalArgumentException("Trying to resolve unknown PhysicalValue: "+name);
        return res;
    }

    public void addPhysicalValue(PhysicalValue value) {
        if (physicalValues.containsKey(value.name)) throw new IllegalArgumentException("Double register for PhysicalValue: "+value.name);
        physicalValues.put(value.name, value);
    }

    @Override
    public String getKey() {
        return CONTEXT_KEY;
    }
}
