/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.dynamicinterface;

import java.util.Vector;

import de.rwth.montisim.commons.utils.json.*;
import de.rwth.montisim.commons.utils.json.JsonTraverser.ArrayIterable;
import de.rwth.montisim.commons.utils.json.JsonTraverser.Entry;
import de.rwth.montisim.commons.utils.json.JsonTraverser.ObjectIterable;

@Typed("struct")
public class StructType extends DataType implements CustomJson {
    public String name;
    public Vector<String> fieldNames = new Vector<>();
    public Vector<DataType> fieldTypes = new Vector<>();

    public StructType(String name) {
        this.name = name;
    }

    private StructType() {
    }

    /**
     * Adds a field to the struct type.
     */
    public void addField(String fieldName, DataType fieldType) {
        fieldNames.add(fieldName);
        fieldTypes.add(fieldType);
    }

    @Override
    public String toString() {
        String res = name + "{ ";
        boolean first = true;
        for (int i = 0; i < fieldNames.size(); ++i) {
            if (first) {
                first = false;
            } else {
                res += ", ";
            }
            res += fieldNames.elementAt(i) + ": " + fieldTypes.elementAt(i);
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
        for (int i = 0; i < fieldNames.size(); ++i) {
            result = prime * result + fieldNames.elementAt(i).hashCode();
            result = prime * result + fieldTypes.elementAt(i).hashCode();
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
        if (this.fieldNames.size() != s.fieldNames.size())
            return false;
        for (int i = 0; i < fieldNames.size(); ++i) {
            if (!fieldNames.elementAt(i).equals(s.fieldNames.elementAt(i)))
                return false;
            if (!fieldTypes.elementAt(i).equals(s.fieldTypes.elementAt(i)))
                return false;
        }
        return true;
    }

    @Override
    public int getDataSize(Object o) {
        Object arr[] = (Object[]) o;
        if (arr.length != fieldTypes.size())
            throw new IllegalArgumentException("Struct data-object has wrong number of entries.");
        int size = 0;
        for (int i = 0; i < fieldTypes.size(); ++i) {
            size += fieldTypes.elementAt(i).getDataSize(arr[i]);
        }
        return size;
    }

    @Override
    public void toJson(JsonWriter j, Object o, SerializationContext context) throws SerializationException {
        Object arr[] = (Object[]) o;
        if (arr.length != fieldTypes.size())
            throw new IllegalArgumentException("Struct data-object has wrong number of entries.");
        j.startArray();
        for (int i = 0; i < fieldTypes.size(); ++i) {
            fieldTypes.elementAt(i).toJson(j, arr[i], context);
        }
        j.endArray();
    }

    @Override
    public Object fromJson(JsonTraverser j, SerializationContext context) throws SerializationException {
        Object arr[] = new Object[fieldTypes.size()];
        ArrayIterable it = j.streamArray();
        int i = 0;
        for (DataType t : fieldTypes) {
            if (!it.iterator().hasNext())
                throw new IllegalArgumentException("Struct data-object serialization missing entries.");
            it.iterator().next();
            arr[i] = t.fromJson(j, context);
            ++i;
        }
        return arr;
    }

    @Override
    public void write(JsonWriter w, SerializationContext context) throws SerializationException {
        w.startObject();
        w.write("type", "struct");
        w.write("name", name);

        w.writeKey("fields");
        w.startObject();
        for (int i = 0; i < fieldTypes.size(); ++i) {
            w.writeKey(fieldNames.elementAt(i));
            Json.toJson(w, fieldTypes.elementAt(i), context);
        }
        w.endObject();

        w.endObject();
    }

    @Override
    public void read(JsonTraverser t, ObjectIterable it, SerializationContext context) throws SerializationException {
        fieldNames.clear();
        fieldTypes.clear();
        for (Entry e : it){
            if (e.key.equals("name")){
                name = t.getString().getJsonString();
            } else if (e.key.equals("fields")){
                for (Entry e2 : t.streamObject()){
                    fieldNames.add(e2.key.getJsonString());
                    fieldTypes.add(Json.instantiateFromJson(t, DataType.class, context));
                }
            } else t.unexpected(e);
        }
    }

    @Override
    public Class<?> getArrayType() {
        return Object[].class;
    }
}