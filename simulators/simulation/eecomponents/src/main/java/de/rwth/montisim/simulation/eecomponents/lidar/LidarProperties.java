package de.rwth.montisim.simulation.eecomponents.lidar;

import de.rwth.montisim.commons.utils.BuildContext;
import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.eesimulator.EEComponent;
import de.rwth.montisim.simulation.eesimulator.EEComponentProperties;
import de.rwth.montisim.simulation.eesimulator.EEComponentType;
import de.rwth.montisim.simulation.eesimulator.EESystem;
import de.rwth.montisim.simulation.eesimulator.exceptions.EEMessageTypeException;
import de.rwth.montisim.simulation.environment.world.World;
import de.rwth.montisim.simulation.vehicle.Vehicle;

@Typed(LidarProperties.TYPE)
public class LidarProperties extends EEComponentProperties {
    public static final String TYPE = "lidar";
    public static final String NAME = "Lidar";

    public LidarProperties() {
        this.name = NAME;
    }

    public LidarProperties setName(String name) {
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
        return new Lidar(this, eesystem, context.getObject(World.CONTEXT_KEY), context.getObject(Vehicle.CONTEXT_KEY));
    }
}
