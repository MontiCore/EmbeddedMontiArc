/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eecomponents.navigation;

import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.eesimulator.components.BusUserProperties;
import de.rwth.montisim.simulation.eesimulator.components.EEComponentType;
import de.rwth.montisim.simulation.eesimulator.components.EEEventProcessor;

@Typed(NavigationProperties.TYPE)
public class NavigationProperties extends BusUserProperties {
    public static final String TYPE = "navigation";

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

    @Override
    public EEEventProcessor build(ComponentBuildContext context) {
        // TODO Auto-generated method stub
        return new Navigation(this, context.pathfinding);
    }

}