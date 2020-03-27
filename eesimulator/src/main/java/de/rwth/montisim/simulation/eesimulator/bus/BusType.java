/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.eesimulator.bus;

public enum BusType {

    INSTANT_BUS("InstantBus"),
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
