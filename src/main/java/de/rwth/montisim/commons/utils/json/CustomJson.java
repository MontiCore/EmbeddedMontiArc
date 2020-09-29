package de.rwth.montisim.commons.utils.json;

import de.rwth.montisim.commons.utils.json.JsonTraverser.ObjectIterable;

public interface CustomJson {
    void write(JsonWriter w, SerializationContext context) throws SerializationException;
    void read(JsonTraverser t, ObjectIterable it, SerializationContext context) throws SerializationException;
}