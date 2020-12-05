/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.dynamicinterface;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.util.List;

import de.rwth.montisim.commons.utils.json.*;

/**
 * Reflection class for the types of Messages sent in the simulation.
 */
public abstract class DataType {
    static {
        Json.registerType(BasicType.class);
        Json.registerType(DynVectorType.class);
        Json.registerType(VectorType.class);
        Json.registerType(StructType.class);
    }

    public abstract int getDataSize(Object o);

    @Override
    public abstract int hashCode();
    @Override
    public abstract boolean equals(Object o);

    public abstract void toJson(JsonWriter j, Object o, SerializationContext context) throws SerializationException;
    public abstract Object fromJson(JsonTraverser j, SerializationContext context) throws SerializationException;

    public abstract void toBinary(DataOutputStream os, Object o) throws IOException;
    public abstract Object fromBinary(DataInputStream is) throws IOException;
    public abstract List<String> toString(Object o);

    public abstract Class<?> getArrayType();
}