/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.dynamicinterface;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.text.DecimalFormat;
import java.util.*;

import de.rwth.montisim.commons.utils.*;
import de.rwth.montisim.commons.utils.json.*;

/**
 * The associated object must be of the Java type specified in the 'Type' enum ('c' param).
 * For native types, use the 'valueOf()' method to get the associated object, ex: Double.valueOf(2.0).
 */
@Typed(BasicType.TYPE)
public class BasicType extends DataType {
    public static final String TYPE = "basic";
    public static final BasicType Q = new BasicType(Type.Q);
    public static final BasicType Z = new BasicType(Type.Z);
    public static final BasicType N = new BasicType(Type.N);
    public static final BasicType N1 = new BasicType(Type.N1);
    public static final BasicType C = new BasicType(Type.C);
    public static final BasicType BOOLEAN = new BasicType(Type.BOOLEAN);
    public static final BasicType DOUBLE = Q;
    public static final BasicType INT = Z;
    // public static final BasicType FLOAT = new BasicType(Type.FLOAT);
    // public static final BasicType BYTE = new BasicType(Type.BYTE);
    public static final BasicType EMPTY = new BasicType(Type.EMPTY);

    public static final BasicType VEC2 = new BasicType(Type.VEC2);
    public static final BasicType VEC3 = new BasicType(Type.VEC3);

    static final DecimalFormat df = new DecimalFormat("#.###");

    public BasicType(Type basic_type) {
        this.base_type = basic_type;
    }

    private BasicType() {
    }

    private Type base_type;

    public static enum Type {
        // @JsonEntry("float")
        // FLOAT(Float.class, float[].class, 4),
        Q(Double.class, double[].class, 8), Z(Integer.class, int[].class, 4), N(Integer.class, int[].class, 4),
        N1(Integer.class, int[].class, 4), C(Vec2.class, Object[].class, 16), @JsonEntry("boolean")
        BOOLEAN(Boolean.class, boolean[].class, 1), @JsonEntry("void")
        EMPTY(null, null, 0),
        // Vec2 and Vec3 are special cases of VectorType with length 2/3
        @JsonEntry("vec2")
        VEC2(Vec2.class, Object[].class, 16), @JsonEntry("vec3")
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
    public void toJson(JsonWriter j, Object o, BuildContext context) throws SerializationException {
        if (o == null)
            return;
        Json.toJson(j, o, context);
    }

    @Override
    public Object fromJson(JsonTraverser j, BuildContext context) throws SerializationException {
        if (base_type.c == null)
            return null;
        return Json.instantiateFromJson(j, base_type.c, context);
    }

    @Override
    public Class<?> getArrayType() {
        return base_type.array_c;
    }

    public Type getType() {
        return base_type;
    }

    @Override
    public void toBinary(DataOutputStream os, Object o) throws IOException {
        switch (base_type) {
            case BOOLEAN:
                if ((Boolean)o){
                    os.writeByte(1);
                } else {
                    os.writeByte(0);
                }
                break;
            case C:
            throw new IllegalArgumentException("Unimplemented");
            case EMPTY:
                break;
            case N:
            case N1:
            case Z:
                os.writeInt((Integer) o);
                break;
            case Q:
                os.writeDouble((Double) o);
                break;
            case VEC2:
                Vec2 v2 = (Vec2) o;
                os.writeDouble(v2.x);
                os.writeDouble(v2.y);
                break;
            case VEC3:
                Vec3 v3 = (Vec3) o;
                os.writeDouble(v3.x);
                os.writeDouble(v3.y);
                os.writeDouble(v3.z);
                break;
            default:
                throw new IllegalArgumentException("Missing case");
        }
    }

    @Override
    public Object fromBinary(DataInputStream is) throws IOException {
        switch (base_type) {
            case BOOLEAN:
                return is.readByte() != 0;
            case C:
                throw new IllegalArgumentException("Unimplemented");
            case EMPTY:
                return null;
            case N:
            case N1:
            case Z:
                return is.readInt();
            case Q:
            return is.readDouble();
            case VEC2:
                double x1 = is.readDouble();
                double y1 = is.readDouble();
                return new Vec2(x1, y1);
            case VEC3:
            double x2 = is.readDouble();
            double y2 = is.readDouble();
            double z2 = is.readDouble();
            return new Vec3(x2, y2, z2);
            default:
            throw new IllegalArgumentException("Missing case");
        }
    }

    @Override
    public String toString() {
        return base_type.name();
    }


    @Override
    public List<String> toString(Object o) {
        switch (base_type) {
            case BOOLEAN:
                if ((Boolean)o) return new ArrayList<String>(Arrays.asList("true"));
                else return new ArrayList<String>(Arrays.asList("false"));
            case C: 
                return new ArrayList<String>(Arrays.asList("Unimplemented Type"));
            case EMPTY: 
                return new ArrayList<String>(Arrays.asList("void"));
            case N:
            case N1:
            case Z: 
                return new ArrayList<String>(Arrays.asList(Integer.toString((Integer) o)));
            case Q: 
                return new ArrayList<String>(Arrays.asList(df.format((Double) o)));
            case VEC2:{
                Vec2 v2 = (Vec2) o;
                return new ArrayList<String>(Arrays.asList("[" + df.format(v2.x) + ", " + df.format(v2.y) + "]"));
            }
            case VEC3: {
                Vec3 v3 = (Vec3) o;
                return new ArrayList<String>(Arrays.asList("[" + df.format(v3.x) + ", " + df.format(v3.y) + ", " + df.format(v3.z)+ "]"));
            }
            default:
                throw new IllegalArgumentException("Missing case");
        }
    }

}
