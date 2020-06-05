/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.dynamicinterface;


public class PortInformation {
    public static enum PortDirection {
        INPUT,
        OUTPUT
    }

    private String name;
    private DataType type;
    private PortDirection direction;
    public PortInformation(String name, DataType type, PortDirection dir){
        this.name = name;
        this.type = type;
        this.direction = dir;
    }

    public String getName(){
        return name;
    }

    public DataType getType(){
        return type;
    }
    
    public PortDirection getDirection() {
        return direction;
    }
}