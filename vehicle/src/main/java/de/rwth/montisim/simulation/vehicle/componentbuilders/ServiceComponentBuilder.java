/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.vehicle.componentbuilders;

import java.util.HashMap;

import de.rwth.montisim.simulation.eesimulator.components.EEComponent;
import de.rwth.montisim.simulation.environment.pathfinding.Pathfinding;
import de.rwth.montisim.simulation.vehicle.Vehicle;

public interface ServiceComponentBuilder {
    public static final HashMap<String, ServiceComponentBuilder> serviceBuilderByName = new HashMap<>();
    public static void registerServiceBuilder(String serviceName, ServiceComponentBuilder builder){
        serviceBuilderByName.put(serviceName, builder);
    }
    public static EEComponent buildServiceComponent(ServiceComponentProperties properties, Pathfinding pathfinding, Vehicle vehicle){
        ServiceComponentBuilder builder = serviceBuilderByName.get(properties.serviceType);
        if (builder == null) {
            // TODO error?
            return null;
        }
        return builder.build(properties, pathfinding, vehicle);
    }

    EEComponent build(ServiceComponentProperties properties, Pathfinding pathfinding, Vehicle vehicle);
}