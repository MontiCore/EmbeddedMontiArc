/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.bus;

import de.rwth.montisim.simulation.eesimulator.EEComponentProperties;

public abstract class BusProperties extends EEComponentProperties {
    public static enum BusType {

        CONSTANT_BUS("ConstantBus"),
        FLEXRAY("FlexRay"),
        CAN("CAN");

        private String name;

        private BusType(String name) {
            this.name = name;
        }

        @Override
        public String toString() {
            return this.name;
        }
    }

    @Override
    public boolean canTransferMessages() {
        return true;
    }

    public abstract BusType getBusType();

}