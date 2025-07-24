/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.bus.constant;

import java.time.Duration;

import de.rwth.montisim.commons.utils.BuildContext;
import de.rwth.montisim.commons.utils.json.JsonEntry;
import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.eesimulator.EESystem;
import de.rwth.montisim.simulation.eesimulator.bus.BusProperties;
import de.rwth.montisim.simulation.eesimulator.EEComponentType;
import de.rwth.montisim.simulation.eesimulator.EEComponent;

@Typed(ConstantBusProperties.TYPE)
public class ConstantBusProperties extends BusProperties {
    public static final String TYPE = "constant_bus";

    ConstantBusProperties() {
        this.name = "DefaultBus";
    }

    public enum TransmissionMode {
        /**
         * Messages directly reach their target when sent.
         */
        @JsonEntry("instant")
        INSTANT,
        /**
         * Sends message at the given data rate, ignoring concurrent messages.
         */
        @JsonEntry("constant_rate")
        CONSTANT_RATE,
        /**
         * All messages take "time" seconds to reach their target.
         */
        @JsonEntry("constant_time")
        CONSTANT_TIME
    }

    public TransmissionMode mode = TransmissionMode.INSTANT;

    /**
     * If TransmissionMode.CONSTANT_RATE, contains the transmission rate in
     * bytes/sec.
     */
    public double rate;
    /**
     * If TransmissionMode.CONSTANT_TIME, contains the time to transfer one message.
     */
    public Duration time;

    public ConstantBusProperties setName(String name) {
        this.name = name;
        return this;
    }

    /**
     * Creates a ConstantBus configuration that instantly transmits its message to
     * their targets.
     */
    public static ConstantBusProperties instantBus() {
        ConstantBusProperties r = new ConstantBusProperties();
        r.mode = TransmissionMode.INSTANT;
        r.rate = 0;
        r.time = null;
        return r;
    }

    /**
     * Creates a ConstantBus configuration that sends messages with a given data
     * rate (in bytes/sec) independently of the number of messages sent
     * concurrently.
     *
     * @param rate Transmission data rate in Bytes/Second.
     */
    public static ConstantBusProperties constantRateBus(double rate) {
        ConstantBusProperties r = new ConstantBusProperties();
        r.mode = TransmissionMode.CONSTANT_RATE;
        r.rate = rate;
        r.time = null;
        return r;
    }

    /**
     * Creates a ConstantBus configuration that sends messages with a constant delay
     * independently of the number of messages sent concurrently.
     *
     * @param time Transmission time for any message.
     */
    public static ConstantBusProperties constantTimeBus(Duration time) {
        ConstantBusProperties r = new ConstantBusProperties();
        r.mode = TransmissionMode.CONSTANT_TIME;
        r.rate = 0;
        r.time = time;
        return r;
    }

    @Override
    public BusType getBusType() {
        return BusType.CONSTANT_BUS;
    }

    @Override
    public EEComponentType getGeneralType() {
        return EEComponentType.BUS;
    }

    @Override
    public String getType() {
        return TYPE;
    }

    @Override
    public float routingCost() {
        return 1.1f;
    }

    @Override
    public EEComponent build(EESystem eesystem, BuildContext context) {
        return new ConstantBus(this, eesystem);
    }
}