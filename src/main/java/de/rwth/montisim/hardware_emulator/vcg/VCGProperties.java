/**
 * (c) https://github.com/MontiCore/monticore
 */
package de.rwth.montisim.hardware_emulator.vcg;

import java.time.Duration;

import de.rwth.montisim.commons.utils.json.Json;
import de.rwth.montisim.commons.utils.json.JsonEntry;
import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.eesimulator.components.BusUserProperties;
import de.rwth.montisim.simulation.eesimulator.components.EEComponentType;
import de.rwth.montisim.simulation.eesimulator.components.EEEventProcessor;

@Typed(VCGProperties.TYPE)
public class VCGProperties extends BusUserProperties {
    public static final String TYPE = "vcg";

    static {
        Json.registerType(TCP.class);
    }

    public CommunicationType communication;
    public TimeMode time = TimeMode.MEASURED;
    public Duration cycle_duration = Duration.ofMillis(100);
    public int ref_id = 0;

    static public abstract class CommunicationType {
    }

    @Typed("tcp")
    static public class TCP extends CommunicationType {
        String host;
        int port;
    }

    static public enum TimeMode {
        @JsonEntry("realtime")
        REALTIME, @JsonEntry("measured")
        MEASURED
    }

    @Override
    public EEComponentType getGeneralType() {
        return EEComponentType.COMPUTER;
    }

    @Override
    public String getType() {
        return TYPE;
    }

    @Override
    public EEEventProcessor build(ComponentBuildContext context) {
        try {
            return new VCG(this, context.componentDestroyer);
        } catch (Exception e) {
            e.printStackTrace();
            throw new IllegalStateException("Could not build VCG component.");
        }
    }

    
}