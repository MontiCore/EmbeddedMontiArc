/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.eesimulator.bridge;

import java.time.Duration;

import de.rwth.montisim.simulation.eesimulator.components.EEComponentProperties;
import de.rwth.montisim.simulation.eesimulator.components.EEComponentType;

public class BridgeProperties extends EEComponentProperties {
    protected BridgeProperties() {
        super(EEComponentType.BRIDGE);
    }
    
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
}