/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator;

public class EventConnectInstruction extends ConnectInstruction {

    protected String eventName;

    public EventConnectInstruction(String name) {
        super();
        eventName = name;
    }

    public EventConnectInstruction(String name, Variable variable1, Variable variable2) {
        super(variable1, variable2);
        eventName = name;
    }

    public EventConnectInstruction(String name, Variable variable1, boolean useThis1, Variable variable2, boolean useThis2) {
        super(variable1, useThis1, variable2, useThis2);
        eventName = name;
    }

    public String getEventName() {
        return eventName;
    }

    public void setEventName(String eventName) {
        this.eventName = eventName;
    }

    @Override
    public String getTargetLanguageInstruction() {
        return null;
    }

    @Override
    public boolean isTargetCodeInstruction() {
        return false;
    }

    @Override
    public boolean isExecuteInstruction() {
        return false;
    }
}
