/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.eesimulator.events;

public enum EEEventType {
	MESSAGE_SEND("MessageSend"),
	MESSAGE_RECEIVE("MessageReceive"),
    //KEEP_ALIVE("KeepAlive"),
    EXECUTE("Execute");

    private final String name;

    private EEEventType(String name){
        this.name = name;
    }

    @Override
    public String toString(){
        return this.name;
    }
}