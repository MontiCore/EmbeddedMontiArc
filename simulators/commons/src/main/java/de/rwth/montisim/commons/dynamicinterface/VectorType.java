/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.dynamicinterface;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.util.*;

import de.rwth.montisim.commons.utils.BuildContext;
import de.rwth.montisim.commons.utils.ParsingException;
import de.rwth.montisim.commons.utils.json.*;
import de.rwth.montisim.commons.utils.json.JsonTraverser.ValueType;

/**
 * Represents a Vector of data. The corresponding object type must be a Java
 * array of the given subtype. The serialization format is a JSON array with the
 * serialized entries.
 */
@Typed("vector")
public class VectorType extends DataType {
    private DataType base_type;
    private int size;

    public VectorType(DataType base_type, int size) {
        this.base_type = base_type;
        this.size = size;
    }

    private VectorType() {
    }

    @Override
    public int getDataSize(Object o) {
        // TODO optimize size calls
        int size = 0;
        Object arr[] = (Object[]) o;
        for (Object oi : arr)
            size += base_type.getDataSize(oi);
        return size;
    }

    @Override
    public String toString() {
        return "[" + base_type.toString() + "; " + size + "]";
        // return "<" + base_type.toString() + "; " + size + ">";
    }

    // Implement hashCode & equals to be able to perform hashmap lookup by type &
    // type comparison

    @Override
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        result = prime * result + ((base_type == null) ? 0 : base_type.hashCode());
        result = prime * result + Integer.valueOf(size).hashCode();
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
        VectorType a = ((VectorType) o);
        return this.base_type.equals(a.base_type) && this.size == a.size;
    }

    @Override
    public Object fromJson(JsonTraverser j, BuildContext context) throws SerializationException {
        Class<?> array_c = base_type.getArrayType();
        if (array_c == null)
            return null;
        if (array_c != Object[].class)
            return Json.instantiateFromJson(j, array_c, context);

        Object o[] = new Object[size];
        int i = 0;
        boolean first = true;
        for (ValueType t : j.streamArray()) {
            if (first) {
                first = false;
                long givenSize = j.getLong();
                if (givenSize != size) throw new ParsingException("Size given in serialized VectorType array does not match the expected vector length.");
                continue;
            }
            if (i >= size)
                throw new ParsingException("Too much entries in Array serialization.");
            o[i] = base_type.fromJson(j, context);
            i++;
        }
        if (i < size)
            throw new ParsingException("Missing entries in Array serialization.");
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
        if (arr.length != size)
            throw new IllegalArgumentException("Array length does not match its ArrayType length");
        j.startArray();
        j.writeValue(size);
        for (Object oi : arr) {
            base_type.toJson(j, oi, context);
        }
        j.endArray();
    }

    @Override
    public Class<?> getArrayType() {
        if (base_type instanceof BasicType) {
            return base_type.getArrayType();
        }
        return Object[].class;
    }

    public int getSize() {
        return size;
    }

    public DataType getBaseType() {
        return base_type;
    }

    @Override
    public void toBinary(DataOutputStream os, Object o) throws IOException {
        Class<?> array_c = base_type.getArrayType();
        // Start with size as short (u16)
        if (array_c == double[].class) {
            double[] arr = (double[]) o;
            os.writeShort(arr.length);
            for (int i = 0; i < arr.length; ++i) {
                os.writeDouble(arr[i]);
            }
        } else throw new IllegalArgumentException("Unimplemented");
    }

    @Override
    public Object fromBinary(DataInputStream is) throws IOException {
        Class<?> array_c = base_type.getArrayType();
        if (array_c == double[].class) {
            int length = is.readShort();
            double[] arr = new double[length];
            for (int i = 0; i < length; ++i) {
                arr[i] = is.readDouble();
            }
            return arr;
        } else throw new IllegalArgumentException("Unimplemented");
    }

    @Override
    public List<String> toString(Object o) {
        Class<?> array_c = base_type.getArrayType();
        if (array_c == double[].class) {
            double[] arr = (double[]) o;
            String res = "[";
            boolean first = true;
            for (double d : arr) {
                if (first) first = false;
                else res += ", ";
                res += BasicType.df.format(d);
            }
            res += "]";
            return new ArrayList<String>(Arrays.asList(res));
        } else return new ArrayList<String>(Arrays.asList("Unimplemented VectorType.toString() for array type "+array_c.getSimpleName()));
    }
}