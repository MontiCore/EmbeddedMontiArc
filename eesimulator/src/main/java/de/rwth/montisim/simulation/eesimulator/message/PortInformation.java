/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.message;


public class PortInformation {
    public static enum PortDirection {
        INPUT,
        OUTPUT
    }

    public final MessageInformation msg;
    public final PortDirection direction;
    public final boolean optional;
    public final boolean multipleInputsAllowed;
    public PortInformation(MessageInformation msg, PortDirection dir, boolean multipleInputsAllowed){
        this.msg = msg;
        this.direction = dir;
        this.multipleInputsAllowed = multipleInputsAllowed;
        this.optional = false;
    }
    public PortInformation(MessageInformation msg, PortDirection dir, boolean multipleInputsAllowed, boolean optional){
        this.msg = msg;
        this.direction = dir;
        this.multipleInputsAllowed = multipleInputsAllowed;
        this.optional = optional;
    }
}