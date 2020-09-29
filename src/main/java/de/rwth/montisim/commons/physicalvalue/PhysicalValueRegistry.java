package de.rwth.montisim.commons.physicalvalue;

import java.util.HashMap;

public class PhysicalValueRegistry {
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
}