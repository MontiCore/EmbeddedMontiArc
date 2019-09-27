/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.bus;

public enum BusType {

    INSTANT_BUS("INSTANT_BUS"),
    FLEXRAY("FLEXRAY"),
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
