package de.rwth.montisim.commons.dynamicinterface;

import de.rwth.montisim.commons.utils.*;
import de.rwth.montisim.commons.utils.json.*;

@Typed(BasicType.TYPE)
public class BasicType extends DataType {
    public static final String TYPE = "basic";
    public static final BasicType DOUBLE = new BasicType(Type.DOUBLE);
    public static final BasicType FLOAT = new BasicType(Type.FLOAT);
    public static final BasicType INT = new BasicType(Type.INT);
    public static final BasicType BOOLEAN = new BasicType(Type.BOOLEAN);
    public static final BasicType BYTE = new BasicType(Type.BYTE);
    public static final BasicType EMPTY = new BasicType(Type.EMPTY);

    public static final BasicType VEC2 = new BasicType(Type.VEC2);
    public static final BasicType VEC3 = new BasicType(Type.VEC3);

    public BasicType(Type basic_type) {
        this.base_type = basic_type;
    }

    private BasicType(){}

    public Type base_type;

    public static enum Type {
        @JsonEntry("double")
        DOUBLE(Double.class, double[].class, 8), 
        @JsonEntry("float")
        FLOAT(Float.class, float[].class, 4),
        @JsonEntry("int")
        INT(Integer.class, int[].class, 4), 
        @JsonEntry("byte")
        BYTE(Byte.class, byte[].class, 1),
        @JsonEntry("boolean")
        BOOLEAN(Boolean.class, boolean[].class, 1), 
        @JsonEntry("void")
        EMPTY(null, null, 0),
        @JsonEntry("vec2")
        VEC2(Vec2.class, Object[].class, 16), 
        @JsonEntry("vec3")
        VEC3(Vec3.class, Object[].class, 24);

        Type(Class<?> c, Class<?> array_c, int size) {
            this.c = c;
            this.array_c = array_c;
            this.size = size;
        }

        public final Class<?> c;
        public final Class<?> array_c;
        public final int size;
    }

    @Override
    public int getDataSize(Object o) {
        return base_type.size;
    }

    @Override
    public int hashCode() {
        return base_type.hashCode();
    }

    @Override
    public boolean equals(Object o) {
        if (o == null)
            return false;
        if (o == this)
            return true;
        if (this.getClass() != o.getClass())
            return false;
        return this.base_type == ((BasicType) o).base_type;
    }

    @Override
    public void toJson(JsonWriter j, Object o, SerializationContext context) throws SerializationException {
        if (o == null)
            return;
        Json.toJson(j, o, context);
    }

    @Override
    public Object fromJson(JsonTraverser j, SerializationContext context) throws SerializationException {
        if (base_type.c == null) return null;
        return Json.instantiateFromJson(j, base_type.c, context);
    }

    @Override
    public Class<?> getArrayType() {
        return base_type.array_c;
    }


}