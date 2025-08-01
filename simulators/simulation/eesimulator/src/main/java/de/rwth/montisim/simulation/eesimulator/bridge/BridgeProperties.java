/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.bridge;

import java.time.Duration;

import de.rwth.montisim.commons.utils.BuildContext;
import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.eesimulator.EEComponentProperties;
import de.rwth.montisim.simulation.eesimulator.EEComponentType;
import de.rwth.montisim.simulation.eesimulator.EESystem;
import de.rwth.montisim.simulation.eesimulator.EEComponent;

@Typed(BridgeProperties.TYPE)
public class BridgeProperties extends EEComponentProperties {
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
    public boolean canTransferMessages() {
        return true;
    }

    @Override
    public float routingCost() {
        return 1.5f;
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
    public EEComponent build(EESystem eesystem, BuildContext context) {
        return new Bridge(this, eesystem);
    }

}