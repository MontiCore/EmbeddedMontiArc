/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.dynamicinterface;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.util.List;

import de.rwth.montisim.commons.utils.BuildContext;
import de.rwth.montisim.commons.utils.json.*;

/**
 * Reflection class for the types of Messages sent in the simulation.
 */
public abstract class DataType {

    public abstract int getDataSize(Object o);

    @Override
    public abstract int hashCode();
    @Override
    public abstract boolean equals(Object o);

    public abstract void toJson(JsonWriter j, Object o, BuildContext context) throws SerializationException;
    public abstract Object fromJson(JsonTraverser j, BuildContext context) throws SerializationException;

    public abstract void toBinary(DataOutputStream os, Object o) throws IOException;
    public abstract Object fromBinary(DataInputStream is) throws IOException;
    public abstract List<String> toString(Object o);

    public abstract Class<?> getArrayType();
}