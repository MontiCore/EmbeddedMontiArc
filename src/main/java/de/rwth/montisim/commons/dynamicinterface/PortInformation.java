package de.rwth.montisim.commons.dynamicinterface;

public class PortInformation {
    public static enum PortDirection {
        INPUT,
        OUTPUT
    }

    public String name;
    public DataType type;
    public PortDirection direction;
    public boolean allows_multiple_inputs;
    public boolean optional;

    protected PortInformation() {}
    
    public PortInformation(String name, DataType type, PortDirection dir, boolean multipleInputsAllowed){
        this.name = name;
        this.type = type;
        this.direction = dir;
        this.allows_multiple_inputs = multipleInputsAllowed;
        this.optional = false;
    }
    public PortInformation(String name, DataType type, PortDirection dir, boolean multipleInputsAllowed, boolean optional){
        this.name = name;
        this.type = type;
        this.direction = dir;
        this.allows_multiple_inputs = multipleInputsAllowed;
        this.optional = optional;
    }
}