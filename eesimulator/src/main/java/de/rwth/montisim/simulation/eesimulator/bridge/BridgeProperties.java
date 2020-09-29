/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.bridge;

import java.time.Duration;

import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.eesimulator.components.BusUserProperties;
import de.rwth.montisim.simulation.eesimulator.components.EEComponentType;
import de.rwth.montisim.simulation.eesimulator.components.EEEventProcessor;

@Typed(BridgeProperties.TYPE)
public class BridgeProperties extends BusUserProperties {
    public static final String TYPE = "bridge";

    /**
     * The delay that is added to each event if it is transmitted over the bride.
     */
    public Duration delay;

    public BridgeProperties setName(String name) {
        this.name = name;
        return this;
    }

    /**
     * Returns a Bridge Configuration with instant transmission property.
     */
    public static BridgeProperties instantBridge() {
        BridgeProperties p = new BridgeProperties();
        p.delay = Duration.ZERO;
        return p;
    }

    @Override
    public EEComponentType getGeneralType() {
        return EEComponentType.BRIDGE;
    }

    @Override
    public String getType() {
        return TYPE;
    }

    @Override
    public EEEventProcessor build(ComponentBuildContext context) {
        return new Bridge(this);
    }

}