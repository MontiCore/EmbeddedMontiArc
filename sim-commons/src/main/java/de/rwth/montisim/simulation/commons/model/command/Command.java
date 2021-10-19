/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.commons.model.command;

/**
 * Command sent to a client.
 */
public class Command {

    public final static String CMD_SCREENSHOT = "SCREENSHOT";

    private String type;
    private Object payload;

    public String getType() {
        return type;
    }

    public void setType(String type) {
        this.type = type;
    }

    public Object getPayload() {
        return payload;
    }

    public void setPayload(Object payload) {
        this.payload = payload;
    }

}
