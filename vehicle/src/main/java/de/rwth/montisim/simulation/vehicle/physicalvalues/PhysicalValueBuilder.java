/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.physicalvalues;

import java.util.HashMap;

import de.rwth.montisim.commons.simulation.PhysicalValue;
import de.rwth.montisim.simulation.vehicle.Vehicle;

public interface PhysicalValueBuilder {
    public static final HashMap<String, PhysicalValueBuilder> builderByName = new HashMap<>();
    public static void registerPhysicalValueBuilder(String valueName, PhysicalValueBuilder builder){
        builderByName.put(valueName, builder);
    }
    public static PhysicalValue buildPhysicalValue(String valueName, Vehicle vehicle){
        PhysicalValueBuilder builder = builderByName.get(valueName);
        if (builder == null) {
            // TODO error?
            return null;
        }
        return builder.build(vehicle);
    }

    PhysicalValue build(Vehicle vehicle);
}