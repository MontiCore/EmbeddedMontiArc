/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.sensor;

import java.lang.reflect.InvocationTargetException;
import java.time.Instant;

import de.rwth.montisim.commons.simulation.PhysicalValue;
import de.rwth.montisim.commons.simulation.TimeUpdate;
import de.rwth.montisim.commons.simulation.Updatable;
import de.rwth.montisim.commons.utils.json.CustomJson;
import de.rwth.montisim.commons.utils.json.Json;
import de.rwth.montisim.commons.utils.json.JsonTraverser;
import de.rwth.montisim.commons.utils.json.JsonTraverser.Entry;
import de.rwth.montisim.commons.utils.json.JsonTraverser.ObjectIterable;
import de.rwth.montisim.commons.utils.json.JsonWriter;
import de.rwth.montisim.commons.utils.json.SerializationContext;
import de.rwth.montisim.simulation.eesimulator.components.EEComponent;
import de.rwth.montisim.simulation.eesimulator.exceptions.EEMessageTypeException;
import de.rwth.montisim.simulation.eesimulator.message.MessageInformation;

public class SensorLogic implements Updatable, CustomJson {
    final transient PhysicalValue watchedValue;
    final transient SensorProperties properties;

    Instant nextUpdate = Instant.EPOCH; // Time after which the sensor can send its next value
    Object lastValue = null; // TODO serialize using type

    transient MessageInformation msgInfo;
    transient EEComponent component;

    public SensorLogic(SensorProperties properties, PhysicalValue watchedValue) {
        this.watchedValue = watchedValue;
        this.properties = properties;
    }

    public void init(EEComponent component) throws EEMessageTypeException {
        this.msgInfo = component.addOutput(watchedValue.name, watchedValue.type);
        this.component = component;
    }

    @Override
    public void update(TimeUpdate newTime) {
        if (newTime.oldTime.isAfter(nextUpdate)) {
            this.nextUpdate = newTime.oldTime.plus(properties.update_interval);
            Object readValue = readValue();
            if (!properties.send_only_changed || hasChanged(readValue)) {
                lastValue = readValue;
                component.sendMessage(newTime.oldTime.plus(properties.read_time), msgInfo, readValue);
            }
        }
    }

    private boolean hasChanged(Object value) {
        // TODO
        // !UMath.equalsThreshold(readValue, lastValue, 0.00001f)
        return true;
    }

    // Can be overwritten to add noise, bias, ...
    public Object readValue() {
        return watchedValue.get();
    }

    private static final String K_NEXT_UPDATE = "nextUpdate";
    private static final String K_LAST_VALUE = "lastValue";

    @Override
    public void write(JsonWriter w, SerializationContext context) throws IllegalAccessException {
        w.startObject();
        w.writeKey(K_NEXT_UPDATE);
        Json.toJson(w, nextUpdate, context);
        if (properties.send_only_changed){
            w.writeKey(K_LAST_VALUE);
            watchedValue.type.toJson(w, lastValue, context);
        }
        w.endObject();
    }

    @Override
    public void read(JsonTraverser t, ObjectIterable it, SerializationContext context)
            throws IllegalAccessException, InstantiationException, InvocationTargetException, NoSuchMethodException {
        for (Entry e : t.streamObject()){
            if (e.key.equals(K_NEXT_UPDATE)) nextUpdate = Json.instantiateFromJson(t, Instant.class, context);
            else if (e.key.equals(K_LAST_VALUE)) lastValue = watchedValue.type.fromJson(t, context);
            else t.unexpected(e);
        }
    }
}