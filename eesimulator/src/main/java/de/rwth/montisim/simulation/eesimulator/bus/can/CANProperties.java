/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.bus.can;

import de.rwth.montisim.simulation.eesimulator.bus.BusProperties;

public class CANProperties extends BusProperties {
    public CANProperties() {
        super(BusType.CAN);
    }

    public CANProperties setBitRate(long bitRate){
        this.bitRate = bitRate;
        return this;
    }

    public CANProperties setName(String name) {
        this.name = name;
        return this;
    }
    
    public long bitRate = 10_000_000; // In bits/sec
}