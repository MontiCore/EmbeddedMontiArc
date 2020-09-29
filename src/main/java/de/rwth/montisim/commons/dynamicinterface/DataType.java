/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.dynamicinterface;

import de.rwth.montisim.commons.utils.json.*;

/**
 * Reflection class for the types of Messages sent in the simulation.
 */
public abstract class DataType {
    static {
        try {
            Json.registerType(BasicType.class);
            Json.registerType(VectorType.class);
            Json.registerType(ArrayType.class);
            Json.registerType(StructType.class);
        } catch (SerializationException e) {
            e.printStackTrace();
            System.exit(-1);
        }
    }

    public abstract int getDataSize(Object o);

    @Override
    public abstract int hashCode();
    @Override
    public abstract boolean equals(Object o);

    public abstract void toJson(JsonWriter j, Object o, SerializationContext context) throws SerializationException;
    public abstract Object fromJson(JsonTraverser j, SerializationContext context) throws SerializationException;

    public abstract Class<?> getArrayType();
}