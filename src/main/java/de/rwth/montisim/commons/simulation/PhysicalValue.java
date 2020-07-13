/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.simulation;

import java.lang.reflect.InvocationTargetException;

import de.rwth.montisim.commons.dynamicinterface.DataType;
import de.rwth.montisim.commons.utils.json.CustomJson;
import de.rwth.montisim.commons.utils.json.JsonTraverser;
import de.rwth.montisim.commons.utils.json.JsonTraverser.ObjectIterable;
import de.rwth.montisim.commons.utils.json.JsonWriter;
import de.rwth.montisim.commons.utils.json.SerializationContext;

/**
 * Wrapper around an Object that represents a physical value inside the
 * simulation. The goal is that multiple users can read and set the value. The
 * getter can be overloaded to hide more complex reading mechanisms for the
 * users.
 */
public class PhysicalValue implements CustomJson {
    protected Object value;
    public final transient String name;
    public final transient DataType type;

    public PhysicalValue(String name, DataType type, Object startValue) {
        this.name = name;
        this.type = type;
        this.value = startValue;
    }

    // These 2 functions can be overwritten to add some transformation on how to
    // set/get the value.
    public Object get() {
        return value;
    }

    public void set(Object value) {
        this.value = value;
    }

    /**
     * This function returns returns the internal value. It should not be
     * overwritten.
     */
    public Object value() {
        return value;
    }

    @Override
    public void write(JsonWriter w, SerializationContext context) throws IllegalAccessException {
        type.toJson(w, value, context);
    }

    @Override
    public void read(JsonTraverser t, ObjectIterable it, SerializationContext context)
            throws IllegalAccessException, InstantiationException, InvocationTargetException, NoSuchMethodException {
        value = type.fromJson(t, context);
    }
}