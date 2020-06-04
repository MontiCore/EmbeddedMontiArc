/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.componentbuilders;

import java.util.HashMap;

import de.rwth.montisim.simulation.eesimulator.components.EEComponent;

public interface ComputerComponentBuilder {
    public static final HashMap<String, ComputerComponentBuilder> serviceBuilderByName = new HashMap<>();
    public static void registerComputerBuilder(String computerName, ComputerComponentBuilder builder){
        serviceBuilderByName.put(computerName, builder);
    }
    public static EEComponent buildComputerComponent(ComputerComponentProperties properties){
        ComputerComponentBuilder builder = serviceBuilderByName.get(properties.computerType);
        if (builder == null) {
            // TODO error?
            return null;
        }
        return builder.build(properties);
    }

    EEComponent build(ComputerComponentProperties properties);
}