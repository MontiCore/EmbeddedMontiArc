/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.eesimulator.message;


public class PortInformation {
    public static enum PortDirection {
        INPUT,
        OUTPUT
    }

    public final MessageInformation msg;
    public final PortDirection direction;
    public final boolean optional;
    public PortInformation(MessageInformation msg, PortDirection dir){
        this.msg = msg;
        this.direction = dir;
        this.optional = false;
    }
    public PortInformation(MessageInformation msg, PortDirection dir, boolean optional){
        this.msg = msg;
        this.direction = dir;
        this.optional = optional;
    }
}