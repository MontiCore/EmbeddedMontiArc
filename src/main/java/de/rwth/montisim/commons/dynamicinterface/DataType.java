/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.dynamicinterface;

import java.lang.reflect.InvocationTargetException;

import de.rwth.montisim.commons.utils.*;
import de.rwth.montisim.commons.utils.json.Json;
import de.rwth.montisim.commons.utils.json.JsonEntry;
import de.rwth.montisim.commons.utils.json.JsonTraverser;
import de.rwth.montisim.commons.utils.json.JsonWriter;
import de.rwth.montisim.commons.utils.json.SerializationContext;

/**
 * Reflection class for the types of Messages sent in the simulation.
 */
public class DataType {
    public static final DataType DOUBLE = new DataType(Type.DOUBLE);
    public static final DataType FLOAT = new DataType(Type.FLOAT);
    public static final DataType INT = new DataType(Type.INT);
    public static final DataType BOOLEAN = new DataType(Type.BOOLEAN);
    public static final DataType BYTE = new DataType(Type.BYTE);
    public static final DataType EMPTY = new DataType(Type.EMPTY);

    public static final DataType VEC2 = new DataType(Type.VEC2);
    public static final DataType VEC3 = new DataType(Type.VEC3);

    public static enum Type {
        // @JsonEntry
        DOUBLE(Double.class, double[].class, "double", 8), 
        FLOAT(Float.class, float[].class, "float", 4), 
        INT(Integer.class, int[].class, "int", 4), 
        BYTE(Byte.class, byte[].class, "byte", 1), 
        BOOLEAN(Boolean.class, boolean[].class, "boolean", 1),
        EMPTY(null, null, "void", 0), 
        VEC2(Vec2.class, Vec2[].class, "Vec2", 16), 
        VEC3(Vec3.class, Vec3[].class, "Vec3", 24), 
        STRUCT(null, null, "struct", -1), 
        ARRAY(null, null, "array", -1);

        Type(Class<?> c, Class<?> arrayC, String name, int size) {
            this.c = c;
            this.arrayC = arrayC;
            this.name = name;
            this.size = size;
        }

        public final Class<?> c;
        public final Class<?> arrayC;
        public final String name;
        public final int size;
    }

    @Override
    public String toString() {
        return type.name;
    }

    public int getDataSize() {
        return type.size;
    }

    public Type type;
    /// The virtual message size in bytes
    // public int dataSize;

    public DataType(Type type) {
        this.type = type;
        // this.dataSize = getDataSize();
    }

    // protected DataType(Type type, int dataSize){
    // this.type = type;
    // this.dataSize = dataSize;
    // }

    @Override
    public int hashCode() {
        return type.hashCode();
    }

    @Override
    public boolean equals(Object o) {
        if (o == null)
            return false;
        if (o == this)
            return true;
        if (this.getClass() != o.getClass())
            return false;
        return this.type == ((DataType) o).type;
    }

    public void toJson(JsonWriter j, Object o, SerializationContext context) throws IllegalArgumentException, IllegalAccessException {
        if (o == null)
            return;
        Json.toJson(j, o, context);
    }

    public Object fromJson(JsonTraverser j, SerializationContext context) throws InstantiationException, IllegalAccessException,
            IllegalArgumentException, InvocationTargetException, NoSuchMethodException, SecurityException {
        if (type.c == null) return null;
        return Json.instantiateFromJson(j, type.c, context);
    }
}