/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.navigation;

import de.rwth.montisim.commons.map.Pathfinding;
import de.rwth.montisim.commons.utils.BuildContext;
import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.eesimulator.EEComponentProperties;
import de.rwth.montisim.simulation.eesimulator.EEComponentType;
import de.rwth.montisim.simulation.eesimulator.EESystem;
import de.rwth.montisim.simulation.eesimulator.exceptions.EEMessageTypeException;
import de.rwth.montisim.simulation.eesimulator.EEComponent;

@Typed(NavigationProperties.TYPE)
public class NavigationProperties extends EEComponentProperties {
    public static final String TYPE = "navigation";
    public static final String NAME = "Navigation";

    public NavigationProperties() {
        this.name = NAME;
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
    public EEComponent build(EESystem eesystem, BuildContext context) throws EEMessageTypeException {
        Pathfinding pathfinding = context.getObject(Pathfinding.CONTEXT_KEY);
        return new Navigation(this, eesystem, pathfinding);
    }

}