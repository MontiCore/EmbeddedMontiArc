/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eecomponents.navigation;

import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.eesimulator.components.EEComponentProperties;
import de.rwth.montisim.simulation.eesimulator.components.EEComponentType;
import de.rwth.montisim.simulation.vehicle.VehicleBuilder;

@Typed(NavigationProperties.TYPE)
public class NavigationProperties extends EEComponentProperties {
    public static final String TYPE = "navigation";
    static {
        VehicleBuilder.registerComponentBuilder(TYPE,
                (properties, context) -> new Navigation((NavigationProperties) properties, context.pathfinding));
    }

    public NavigationProperties() {
        this.name = "DefaultNavigation";
    }

    public NavigationProperties setName(String name) {
        this.name = name;
        return this;
    }

    @Override
    public EEComponentType getGeneralType() {
        return EEComponentType.SERVICE;
    }

    @Override
    public String getType() {
        return TYPE;
    }

}