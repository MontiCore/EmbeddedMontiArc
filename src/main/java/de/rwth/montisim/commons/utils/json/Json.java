package de.rwth.montisim.commons.utils.json;

import java.io.File;
import java.io.IOException;
import java.lang.reflect.Array;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Modifier;
import java.util.Iterator;

import de.rwth.montisim.commons.utils.JsonTraverser;
import de.rwth.montisim.commons.utils.JsonWriter;
import de.rwth.montisim.commons.utils.ParsingException;
import de.rwth.montisim.commons.utils.JsonTraverser.ArrayIterable;
import de.rwth.montisim.commons.utils.JsonTraverser.Entry;
import de.rwth.montisim.commons.utils.JsonTraverser.ValueType;

// TODO handle exceptions locally ?
public class Json {
    public static final String K_TYPE = "type";
    JsonWriter w;
    JsonTraverser t;

    protected Json() {
    }

    public static Json formattedWriter() {
        Json j = new Json();
        j.w = new JsonWriter();
        j.w.init();
        j.w.format = true;
        return j;
    }

    public static Json traverser(String json) throws IOException {
        Json j = new Json();
        j.t = new JsonTraverser();
        j.t.init(json);
        return j;
    }

    public static Json traverser(File json) throws IOException {
        Json j = new Json();
        j.t = new JsonTraverser();
        j.t.init(json);
        return j;
    }

    public void toJson(Object o) throws IllegalArgumentException, IllegalAccessException {
        Class<?> c = o.getClass();
        if (basicTypeToJson(o, c))
            return;

        Jsonable ja = c.getDeclaredAnnotation(Jsonable.class);
        if (ja == null)
            throw new IllegalArgumentException("Object to be serialized has no 'Jsonable' annotation.");
        fieldsToJson(o, c, ja, true);

    }


    public <T> T instanciateFromJson(Class<T> c) throws InstantiationException, IllegalAccessException,
            IllegalArgumentException, InvocationTargetException, NoSuchMethodException, SecurityException {
        T o = (T) instantiateBasicType(c);
        if (o != null)
            return o;
        Jsonable ja = c.getDeclaredAnnotation(Jsonable.class);
        if (ja == null)
            throw new IllegalArgumentException("Object to be instantiated has no 'Jsonable' annotation.");
            
        o = c.getDeclaredConstructor().newInstance();
        
        Typed ty = c.getDeclaredAnnotation(Typed.class);
        boolean typed = ty != null;
        boolean obj = ja.type() == StructureType.OBJECT;
        boolean all = ja.fields() == FieldSelect.ALL;
        FieldOptional fo = ja.optional();
        
        if (obj){
            Field[] fields = c.getDeclaredFields();
            int fieldCount = fields.length;
    
            String[] keys = new String[fieldCount];
    
            for (int i = 0; i < fields.length; ++i){
                Field f = fields[i];
                if (!isValid(f)) continue;
                JsonEntry je = f.getAnnotation(JsonEntry.class);
                if (!isUsed(je, all)) continue;

                String key = je != null ? je.value() : null;
                if (key == null || key.length() == 0) key = f.getName();
                keys[i] = key;
            }

            for (Entry e : t.streamObject()){
                int kid = 0;
                for (; kid < keys.length; ++kid){
                    if (keys[kid] != null && e.key.equals(keys[kid])) break;
                }
                if (kid == keys.length) t.unexpected(e);
                Field f = fields[kid];
                Class<?> fc = f.getType();
                if (fc.isPrimitive()) readPrimitiveField(fc, f, o);
                else f.set(o, instanciateFromJson(fc));
            }
        } else {
            Field[] fields = c.getDeclaredFields();
            Field[] entries = new Field[fields.length];
            int eCount = 0;
            for (int i = 0; i < fields.length; ++i){
                Field f = fields[i];
                if (!isValid(f)) continue;
                JsonEntry je = f.getAnnotation(JsonEntry.class);
                if (!isUsed(je, all)) continue;
                entries[eCount++] = f;
            }
            int i = 0;
            for (ValueType v : t.streamArray()){
                if (i >= eCount) throw new ParsingException("Too many entries in "+c.getSimpleName()+" array");
                Field f = entries[i];
                Class<?> fc = f.getType();
                if (fc.isPrimitive()) readPrimitiveField(fc, f, o);
                else f.set(o, instanciateFromJson(fc));
                ++i;
            }
            if (i < eCount) throw new ParsingException("Missing entries in "+c.getSimpleName()+" array");
        }
        return o;
    }

    private boolean isValid(Field f){
        int mod = f.getModifiers();
        if (Modifier.isStatic(mod) || Modifier.isTransient(mod)) return false;
        if (Modifier.isPrivate(mod)) f.setAccessible(true);
        return true;
    }

    private boolean isUsed(JsonEntry je, boolean all){
        return je != null || all;
    }

    private void fieldsToJson(Object o, Class<?> c, Jsonable ja, boolean first)
            throws IllegalArgumentException, IllegalAccessException {
        Typed t = c.getDeclaredAnnotation(Typed.class);
        boolean typed = t != null;
        boolean obj = ja.type() == StructureType.OBJECT;
        boolean all = ja.fields() == FieldSelect.ALL;
        FieldOptional fo = ja.optional();

        if (first){
            if (obj) w.startObject();
            else w.startArray();
    
            if (typed){
                String type = t.value();
                if (type.length() == 0) type = c.getSimpleName();
                if (obj) w.writeKey(K_TYPE);
                w.writeValue(type);
            }
        }
        

        Class<?> superC = c.getSuperclass();
        Jsonable superJa = superC.getDeclaredAnnotation(Jsonable.class);
        if (superJa != null) fieldsToJson(o, superC, superJa, false);
        

        for(Field f : c.getDeclaredFields()){
            if (!isValid(f)) continue;
            JsonEntry je = f.getAnnotation(JsonEntry.class);
            if (!isUsed(je, all)) continue;

            if (obj){
                String name = je != null ? je.value() : null;
                if (name == null || name.length() == 0) name = f.getName();
                w.writeKey(name);
            }
            
            Class<?> fc = f.getType();
            if (fc.isPrimitive()) writePrimitiveField(fc, f, o);
            else toJson(f.get(o));
        }
        if(first){
            if (obj) w.endObject();
            else w.endArray();
        }
    }

    private void writePrimitiveField(Class<?> fc,Field f, Object o)
            throws IllegalArgumentException, IllegalAccessException {
        if (fc.equals(double.class)) {
            w.writeValue(f.getDouble(o));
        } else if (fc.equals(int.class)) {
            w.writeValue(f.getInt(o));
        } else if (fc.equals(float.class)) {
            w.writeValue(f.getFloat(o));
        } else if (fc.equals(char.class)) {
            w.writeValue(f.getChar(o));
        } else if (fc.equals(byte.class)) {
            w.writeValue(f.getByte(o));
        } else if (fc.equals(long.class)) {
            w.writeValue(f.getLong(o));
        } else if (fc.equals(short.class)) {
            w.writeValue(f.getShort(o));
        } else if (fc.equals(boolean.class)) {
            w.writeValue(f.getBoolean(o));
        } else throw new IllegalArgumentException("Unsupported primitive type: "+fc.toString());
    }

    private void readPrimitiveField(Class<?> fc,Field f, Object o)
            throws IllegalArgumentException, IllegalAccessException {
        if (fc.equals(double.class)) {
            f.setDouble(o, t.getDouble());
        } else if (fc.equals(int.class)) {
            f.setInt(o, (int)t.getLong());
        } else if (fc.equals(float.class)) {
            f.setFloat(o, (float)t.getDouble());
        } else if (fc.equals(char.class)) {
            f.setChar(o, (char)t.getLong());
        } else if (fc.equals(byte.class)) {
            f.setByte(o, (byte)t.getLong());
        } else if (fc.equals(long.class)) {
            f.setLong(o, t.getLong());
        } else if (fc.equals(short.class)) {
            f.setShort(o, (short)t.getLong());
        } else if (fc.equals(boolean.class)) {
            f.setBoolean(o, t.getBoolean());
        } else throw new IllegalArgumentException("Unsupported primitive type: "+fc.toString());
    }
    
    private boolean basicTypeToJson(Object o, Class<?> c) throws IllegalArgumentException, IllegalAccessException {
        if (c.equals(String.class)){
            w.writeValue((String)o);
        } else if (Number.class.isAssignableFrom(c)){
            if (c.equals(Double.class)){
                w.writeValue((Double)o);
            } else if (c.equals(Integer.class)){
                w.writeValue((Integer)o);
            } else if (c.equals(Float.class)){
                w.writeValue((Float)o);
            } else if (c.equals(Long.class)){
                w.writeValue((Long)o);
            } else if (c.equals(Short.class)){
                w.writeValue((Short)o);
            } else if (c.equals(Byte.class)){
                w.writeValue((Byte)o);
            } else throw new IllegalArgumentException("Unsupported Number type: "+c.toString());
        }
        else if (c.equals(Character.class)){
            w.writeValue((Character)o);
        }  else if (c.equals(Boolean.class)){
            w.writeValue((Boolean)o);
        } else if (c.isArray()) {
            w.startArray();
            if (c.equals(double[].class)){
                double[] arr = (double[])o;
                w.writeValue(arr.length);
                for (double v : arr) w.writeValue(v);
            } else if (c.equals(float[].class)){
                float[] arr = (float[])o;
                w.writeValue(arr.length);
                for (float v : arr) w.writeValue(v);
            } else if (c.equals(int[].class)){
                int[] arr = (int[])o;
                w.writeValue(arr.length);
                for (int v : arr) w.writeValue(v);
            } else if (c.equals(long[].class)){
                long[] arr = (long[])o;
                w.writeValue(arr.length);
                for (long v : arr) w.writeValue(v);
            } else if (c.equals(short[].class)){
                short[] arr = (short[])o;
                w.writeValue(arr.length);
                for (short v : arr) w.writeValue(v);
            } else if (c.equals(byte[].class)){
                byte[] arr = (byte[])o;
                w.writeValue(arr.length);
                for (byte v : arr) w.writeValue(v);
            } else if (c.equals(char[].class)){
                char[] arr = (char[])o;
                w.writeValue(arr.length);
                for (char v : arr) w.writeValue(v);
            } else if (c.equals(boolean[].class)){
                boolean[] arr = (boolean[])o;
                w.writeValue(arr.length);
                for (boolean v : arr) w.writeValue(v);
            } else {
                Object[] arr = (Object[])o;
                w.writeValue(arr.length);
                for (Object v : arr) toJson(v);
            }
            w.endArray();
        } else return false;
        return true;
    }

    
    private Object instantiateBasicType(Class<?> c) throws InstantiationException, IllegalAccessException,
            IllegalArgumentException, InvocationTargetException, NoSuchMethodException, SecurityException {
        if (c.equals(String.class)){
            return t.getString().getJsonString();
        } else if (Number.class.isAssignableFrom(c)){
            if (c.equals(Double.class)){
                return t.getDouble();
            } else if (c.equals(Integer.class)){
                return (int) t.getLong();
            } else if (c.equals(Float.class)){
                return (float) t.getDouble();
            } else if (c.equals(Long.class)){
                return t.getLong();
            } else if (c.equals(Short.class)){
                return (short) t.getLong();
            } else if (c.equals(Byte.class)){
                return (byte) t.getLong();
            } else throw new IllegalArgumentException("Unsupported Number type: "+c.toString());
        }
        else if (c.equals(Character.class)){
            return (char) t.getLong();
        }  else if (c.equals(Boolean.class)){
            return t.getBoolean();
        } else if (c.isArray()) {
            ArrayIterable it = t.streamArray();
            if (!it.iterator().hasNext()) throw new ParsingException("Expected array length at start of array");
            it.iterator().next();
            int length = (int)t.getLong();
            int i = 0;
            if (c.equals(double[].class)){
                double[] arr = new double[length];
                for (ValueType ty : it) arr[i++] = t.getDouble();
                return arr;
            } else if (c.equals(float[].class)){
                float[] arr = new float[length];
                for (ValueType ty : it) arr[i++] = (float) t.getDouble();
                return arr;
            } else if (c.equals(int[].class)){
                int[] arr = new int[length];
                for (ValueType ty : it) arr[i++] = (int) t.getLong();
                return arr;
            } else if (c.equals(long[].class)){
                long[] arr = new long[length];
                for (ValueType ty : it) arr[i++] = t.getLong();
                return arr;
            } else if (c.equals(short[].class)){
                short[] arr = new short[length];
                for (ValueType ty : it) arr[i++] = (short) t.getLong();
                return arr;
            } else if (c.equals(byte[].class)){
                byte[] arr = new byte[length];
                for (ValueType ty : it) arr[i++] = (byte) t.getLong();
                return arr;
            } else if (c.equals(char[].class)){
                // TODO as String?
                char[] arr = new char[length];
                for (ValueType ty : it) arr[i++] = (char) t.getLong();
                return arr;
            } else if (c.equals(boolean[].class)){
                boolean[] arr = new boolean[length];
                for (ValueType ty : it) arr[i++] = t.getBoolean();
                return arr;
            } else {
                Object[] arr = new Object[length];
                for (ValueType ty : it) arr[i++] = instanciateFromJson(c.getComponentType());
                return arr;
            }
        } else return null;
    }

}