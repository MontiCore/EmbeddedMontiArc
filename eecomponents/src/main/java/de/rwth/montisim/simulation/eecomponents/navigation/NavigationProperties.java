/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.eecomponents.navigation;

import de.rwth.montisim.simulation.environment.pathfinding.Pathfinding;
import de.rwth.montisim.simulation.vehicle.Vehicle;
import de.rwth.montisim.simulation.vehicle.componentbuilders.ServiceComponentBuilder;
import de.rwth.montisim.simulation.vehicle.componentbuilders.ServiceComponentProperties;

public class NavigationProperties extends ServiceComponentProperties {
    public static final String SERVICE_TYPE = "Navigation";
    static {
        ServiceComponentBuilder.registerServiceBuilder(SERVICE_TYPE, 
            (ServiceComponentProperties properties, Pathfinding pathfinding, Vehicle vehicle) -> 
                new Navigation((NavigationProperties) properties, pathfinding)
        );
    }

    public NavigationProperties() {
        super(SERVICE_TYPE);
        this.name = "UnnamedNavigation";
    }
    
}