/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.bus.can;

import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.eesimulator.bus.BusProperties;
import de.rwth.montisim.simulation.eesimulator.components.EEComponentType;
import de.rwth.montisim.simulation.eesimulator.components.EEEventProcessor;

@Typed(CANProperties.TYPE)
public class CANProperties extends BusProperties {
    public static final String TYPE = "can_bus";

    public CANProperties setBitRate(long bitRate) {
        this.bit_rate = bitRate;
        return this;
    }

    public CANProperties setName(String name) {
        this.name = name;
        return this;
    }

    public long bit_rate = 10_000_000; // In bits/sec

    @Override
    public BusType getBusType() {
        return BusType.CAN;
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
    public EEEventProcessor build(ComponentBuildContext context) {
        return new CAN(this, context.comp);
    }
}