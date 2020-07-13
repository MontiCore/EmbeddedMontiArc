/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.events;

import de.rwth.montisim.commons.eventsimulation.DiscreteEvent;
import de.rwth.montisim.commons.utils.ParsingException;
import de.rwth.montisim.commons.utils.StringRef;
import de.rwth.montisim.commons.utils.json.CustomJson;
import de.rwth.montisim.commons.utils.json.Json;
import de.rwth.montisim.commons.utils.json.JsonTraverser;
import de.rwth.montisim.commons.utils.json.JsonWriter;
import de.rwth.montisim.simulation.eesimulator.components.ComponentManager;
import de.rwth.montisim.simulation.eesimulator.components.EEEventProcessor;
import de.rwth.montisim.simulation.eesimulator.message.MessageTypeManager;

import java.lang.reflect.InvocationTargetException;
import java.time.Instant;
import java.util.Optional;

/**
 * Discrete Events that are processed by the EESimulator.
 */
public abstract class EEDiscreteEvent extends DiscreteEvent<EEEventType> {
    private EEEventProcessor target;

    public EEDiscreteEvent(Instant eventTime, EEEventProcessor target) {
        super(eventTime);
        this.target = target;
    }

    protected EEDiscreteEvent() {
        this.target = null;
    }

    public EEEventProcessor getTarget() {
        return target;
    }

    @Override
    public String toString() {
        return "EEDiscreteEvent {type: " + getEventType() + ", target: " + this.target + ", time: " + this.time + "}";
    }

    // public static final String K_TIME = "time";
    // public static final String K_TARGET = "target";

    // protected void write(JsonWriter j, ComponentManager cm, MessageTypeManager mtm) throws IllegalArgumentException, IllegalAccessException {
    //     j.writeKey(K_TIME);
    //     Json.toJson(j, time);
    //     j.write(K_TARGET, target.properties.name);
    // }

    // protected boolean read(JsonTraverser j, StringRef key, ComponentManager cm, MessageTypeManager mtm)
    //         throws InstantiationException, IllegalAccessException, IllegalArgumentException, InvocationTargetException,
    //         NoSuchMethodException, SecurityException {
    //     if (key.equals(K_TIME)){
    //         this.time = Json.instantiateFromJson(j, Instant.class);
    //     } else if (key.equals(K_TARGET)){
    //         String name = j.getString().getJsonString();
    //         Optional<EEEventProcessor> r = cm.getComponent(name);
    //         if (!r.isPresent()) throw new ParsingException("Unkonwn target component: "+name);
    //         this.target = r.get();
    //     } else return false;
    //     return true;
    // }

    // abstract void writeEventData(JsonWriter j, ComponentManager cm, MessageTypeManager mtm);
    // abstract void readEventData(JsonTraverser j, StringRef key, ComponentManager cm, MessageTypeManager mtm);

    public static abstract class EventData {
        Instant time;
        String target;

        EventData(EEDiscreteEvent event) {
            this.time = event.time;
            this.target = event.target.properties.name;
        }

        public abstract EEDiscreteEvent getEvent(ComponentManager cm);
    }
    public abstract EventData getEventData();
    protected static EEEventProcessor getTarget(String name, ComponentManager cm){
        Optional<EEEventProcessor> r = cm.getComponent(name);
        if (!r.isPresent()) throw new ParsingException("Unknown target component: "+name);
        return r.get();
    }
}