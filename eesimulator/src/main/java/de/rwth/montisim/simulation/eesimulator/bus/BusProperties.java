/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.eesimulator.bus;

import de.rwth.montisim.simulation.eesimulator.components.EEComponentProperties;
import de.rwth.montisim.simulation.eesimulator.components.EEComponentType;

public abstract class BusProperties extends EEComponentProperties {
    public static enum BusType {

        CONSTANT_BUS("ConstantBus"),
        FLEXRAY("FlexRay"),
        CAN("CAN");
    
        private String name;
    
        private BusType(String name){
            this.name = name;
        }
    
        @Override
        public String toString(){
            return this.name;
        }
    }
    
    public final BusType busType;

    public BusProperties(BusType busType) {
        super(EEComponentType.BUS);
        this.busType = busType;
    }
    
}