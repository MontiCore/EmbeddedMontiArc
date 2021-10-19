package de.rwth.montisim.simulation.eecomponents.speed_limit;

import de.rwth.montisim.commons.utils.BuildContext;
import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.eesimulator.EEComponent;
import de.rwth.montisim.simulation.eesimulator.EEComponentProperties;
import de.rwth.montisim.simulation.eesimulator.EEComponentType;
import de.rwth.montisim.simulation.eesimulator.EESystem;
import de.rwth.montisim.simulation.eesimulator.exceptions.EEMessageTypeException;
import de.rwth.montisim.simulation.environment.world.World;

@Typed(SpeedLimitServiceProperties.TYPE)
public class SpeedLimitServiceProperties extends EEComponentProperties {
    public static final String TYPE = "speed_limit";
    public static final String NAME = "SpeedLimit";

    public SpeedLimitServiceProperties() {
        this.name = NAME;
    }

    public SpeedLimitServiceProperties setName(String name) {
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
        return new SpeedLimitService(this, eesystem, context.getObject(World.CONTEXT_KEY));
    }

}
