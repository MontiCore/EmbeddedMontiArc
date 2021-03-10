/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.dynamicinterface;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.util.*;

import de.rwth.montisim.commons.utils.BuildContext;
import de.rwth.montisim.commons.utils.json.*;
import de.rwth.montisim.commons.utils.json.JsonTraverser.ArrayIterable;
import de.rwth.montisim.commons.utils.json.JsonTraverser.Entry;
import de.rwth.montisim.commons.utils.json.JsonTraverser.ObjectIterable;

/**
 * Represents a named Struct with named and typed field. The corresponding Java
 * object type must be an array of objects, in declaration order of the struct
 * fields. Each object in the array must respect the data format for the field
 * type. The serialization format for struct types is a JSON array with the
 * serialization of each field, in declaration order.
 */
@Typed(StructType.TYPE)
public class StructType extends DataType implements CustomJson {
    public static final String TYPE = "struct";
    private String name;
    private Vector<String> field_names = new Vector<>();
    private Vector<DataType> field_types = new Vector<>();

    public StructType(String name) {
        this.name = name;
    }

    private StructType() {
    }

    /**
     * Adds a field to the struct type.
     */
    public void addField(String fieldName, DataType fieldType) {
        field_names.add(fieldName);
        field_types.add(fieldType);
    }

    public String getName() {
        return name;
    }

    public int getFieldCount() {
        return field_names.size();
    }

    public String getFieldName(int i) {
        return field_names.elementAt(i);
    }

    public DataType getFieldType(int i) {
        return field_types.elementAt(i);
    }

    @Override
    public String toString() {
        String res = name + "{ ";
        boolean first = true;
        for (int i = 0; i < field_names.size(); ++i) {
            if (first) {
                first = false;
            } else {
                res += ", ";
            }
            res += field_names.elementAt(i) + ": " + field_types.elementAt(i);
        }
        res += "}";
        return res;
    }

    // Implement hashCode & equals to be able to perform hashmap lookup by type &
    // type comparison

    @Override
    public int hashCode() {
        final int prime = 31;
        int result = name.hashCode();
        for (int i = 0; i < field_names.size(); ++i) {
            result = prime * result + field_names.elementAt(i).hashCode();
            result = prime * result + field_types.elementAt(i).hashCode();
        }
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
        StructType s = ((StructType) o);
        if (!this.name.equals(s.name))
            return false;
        if (this.field_names.size() != s.field_names.size())
            return false;
        for (int i = 0; i < field_names.size(); ++i) {
            if (!field_names.elementAt(i).equals(s.field_names.elementAt(i)))
                return false;
            if (!field_types.elementAt(i).equals(s.field_types.elementAt(i)))
                return false;
        }
        return true;
    }

    @Override
    public int getDataSize(Object o) {
        Object arr[] = (Object[]) o;
        if (arr.length != field_types.size())
            throw new IllegalArgumentException("Struct data-object has wrong number of entries.");
        int size = 0;
        for (int i = 0; i < field_types.size(); ++i) {
            size += field_types.elementAt(i).getDataSize(arr[i]);
        }
        return size;
    }

    @Override
    public void toJson(JsonWriter j, Object o, BuildContext context) throws SerializationException {
        Object arr[] = (Object[]) o;
        if (arr.length != field_types.size())
            throw new IllegalArgumentException("Struct data-object has wrong number of entries.");
        j.startArray();
        for (int i = 0; i < field_types.size(); ++i) {
            field_types.elementAt(i).toJson(j, arr[i], context);
        }
        j.endArray();
    }

    @Override
    public Object fromJson(JsonTraverser j, BuildContext context) throws SerializationException {
        Object arr[] = new Object[field_types.size()];
        ArrayIterable it = j.streamArray();
        int i = 0;
        for (DataType t : field_types) {
            if (!it.iterator().hasNext())
                throw new IllegalArgumentException("Struct data-object serialization missing entries.");
            it.iterator().next();
            arr[i] = t.fromJson(j, context);
            ++i;
        }
        return arr;
    }

    @Override
    public void write(JsonWriter w, BuildContext context) throws SerializationException {
        w.startObject();
        w.write("type", TYPE);
        w.write("name", name);

        w.writeKey("fields");
        w.startObject();
        for (int i = 0; i < field_types.size(); ++i) {
            w.writeKey(field_names.elementAt(i));
            Json.toJson(w, field_types.elementAt(i), context);
        }
        w.endObject();

        w.endObject();
    }

    @Override
    public void read(JsonTraverser t, ObjectIterable it, BuildContext context) throws SerializationException {
        field_names.clear();
        field_types.clear();
        for (Entry e : it) {
            if (e.key.equals("name")) {
                name = t.getString().getJsonString();
            } else if (e.key.equals("fields")) {
                for (Entry e2 : t.streamObject()) {
                    field_names.add(e2.key.getJsonString());
                    field_types.add(Json.instantiateFromJson(t, DataType.class, context));
                }
            } else
                t.unexpected(e);
        }
    }

    @Override
    public Class<?> getArrayType() {
        return Object[].class;
    }

    @Override
    public void toBinary(DataOutputStream os, Object o) throws IOException {
        Object[] arr = (Object[]) o;
        for(int i=0;i<this.getFieldCount();i++){
            field_types.elementAt(i).toBinary(os, arr[i]);
        }
    }

    @Override
    public Object fromBinary(DataInputStream is) throws IOException {
        Object[] arr = new Object[this.getFieldCount()];
        for(int i=0;i<arr.length;i++){
            arr[i] = this.field_types.elementAt(i).fromBinary(is);
        }
        return (Object) arr;
    }

    @Override
    public List<String> toString(Object o) {
        return new ArrayList<String>(Arrays.asList("Unimplemented toString()"));
    }

}