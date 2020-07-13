package de.rwth.montisim.commons.utils.json;

import java.lang.reflect.InvocationTargetException;

import de.rwth.montisim.commons.utils.json.JsonTraverser.ObjectIterable;

public interface CustomJson {
    void write(JsonWriter w, SerializationContext context) throws IllegalAccessException;
    void read(JsonTraverser t, ObjectIterable it, SerializationContext context) throws IllegalAccessException, InstantiationException, InvocationTargetException, NoSuchMethodException;
}