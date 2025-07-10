/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.dynamicinterface;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import de.rwth.montisim.commons.utils.BuildContext;
import de.rwth.montisim.commons.utils.ParsingException;
import de.rwth.montisim.commons.utils.json.*;
import de.rwth.montisim.commons.utils.json.JsonTraverser.ArrayIterable;
import de.rwth.montisim.commons.utils.json.JsonTraverser.ValueType;

@Typed("dyn_vector")
public class DynVectorType extends DataType {
    public DataType base_type;
    public int max_size;

    public DynVectorType(DataType base_type, int max_size) {
        this.base_type = base_type;
        this.max_size = max_size;
    }

    private DynVectorType() {
    }

    @Override
    public int getDataSize(Object o) {
        // TODO optimize size calls
        int size = 0;
        Object arr[] = (Object[]) o;
        for (Object oi : arr)
            size += base_type.getDataSize(oi);
        return size + 4;
    }

    @Override
    public String toString() {
        return "<" + base_type.toString() + "; " + max_size + ">";
    }

    // Implement hashCode & equals to be able to perform hashmap lookup by type &
    // type comparison

    @Override
    public int hashCode() {
        final int prime = 13;
        int result = 1;
        result = prime * result + ((base_type == null) ? 0 : base_type.hashCode());
        result = prime * result + Integer.valueOf(max_size).hashCode();
        return result;
    }

    @Override
    public boolean equals(Object o) {
        if (o == null)
            return false;
        if (o == this)
            return true;
        if (this.getClass() != o.getClass())
            return false;
        DynVectorType a = ((DynVectorType) o);
        return this.base_type.equals(a.base_type) && this.max_size == a.max_size;
    }

    @Override
    public Object fromJson(JsonTraverser j, BuildContext context) throws SerializationException {
        Class<?> array_c = base_type.getArrayType();
        if (array_c == null)
            return null;
        if (array_c != Object[].class)
            return Json.instantiateFromJson(j, array_c, context);
        ArrayIterable it = j.streamArray();
        if (!it.iterator().hasNext())
            throw new ParsingException("Missing length entry in VectorType serialization.");
        it.iterator().next();
        int size = (int) j.getLong();
        Object o[] = new Object[size];
        int i = 0;
        for (ValueType t : it) {
            if (i >= size)
                throw new ParsingException("Too much entries in VectorType serialization.");
            o[i] = base_type.fromJson(j, context);
            i++;
        }
        if (i < size)
            throw new ParsingException("Missing entries in VectorType serialization.");
        return o;
    }

    @Override
    public void toJson(JsonWriter j, Object o, BuildContext context) throws SerializationException {
        if (o == null)
            return;
        Class<?> array_c = base_type.getArrayType();
        if (array_c == null)
            return;
        if (array_c != Object[].class) {
            Json.toJson(j, o, context);
            return;
        }

        Object arr[] = (Object[]) o;
        j.startArray();
        if (arr.length > max_size)
            throw new IllegalArgumentException("Vector length above VectorType limit.");
        j.writeValue(arr.length);
        for (Object oi : arr) {
            base_type.toJson(j, oi, context);
        }
        j.endArray();
    }

    @Override
    public Class<?> getArrayType() {
        return Object[].class;
    }

    @Override
    public void toBinary(DataOutputStream os, Object o) throws IOException {
        throw new IllegalArgumentException("Unimplemented");
    }

    @Override
    public Object fromBinary(DataInputStream is) throws IOException {
        throw new IllegalArgumentException("Unimplemented");
    }

    @Override
    public List<String> toString(Object o) {
        return new ArrayList<String>(Arrays.asList("Unimplemented toString()"));
    }

}