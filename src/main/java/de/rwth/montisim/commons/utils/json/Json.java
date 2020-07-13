package de.rwth.montisim.commons.utils.json;

import java.io.File;
import java.io.IOException;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Modifier;
import java.lang.reflect.ParameterizedType;
import java.time.Duration;
import java.time.Instant;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Optional;
import java.util.Stack;
import java.util.Vector;

import de.rwth.montisim.commons.utils.*;
import de.rwth.montisim.commons.utils.json.JsonTraverser.*;

// TODO handle exceptions locally ?
public abstract class Json {
    public static final String K_TYPE = "type";

    // JsonWriter w;
    // JsonTraverser t;

    protected Json() {
    }

    public static String toFormattedJson(Object o, SerializationContext context)
            throws IllegalArgumentException, IllegalAccessException {
        JsonWriter w = new JsonWriter(true);
        Json.getTypeInfo(o.getClass()).writer.write(w, o, context);
        return w.getString();
    }

    public static String toFormattedJson(Object o) throws IllegalArgumentException, IllegalAccessException {
        return toFormattedJson(o, null);
    }

    public static String toJson(Object o, SerializationContext context)
            throws IllegalArgumentException, IllegalAccessException {
        JsonWriter w = new JsonWriter(false);
        toJson(w, o, context);
        return w.getString();
    }

    public static String toJson(Object o) throws IllegalArgumentException, IllegalAccessException {
        return toJson(o, null);
    }

    public static void toJson(JsonWriter w, Object o, SerializationContext context)
            throws IllegalArgumentException, IllegalAccessException {
        Json.getTypeInfo(o.getClass()).writer.write(w, o, context);
    }

    public static <T> T instantiateFromJson(String json, Class<T> c, SerializationContext context)
            throws InstantiationException, IllegalAccessException, IllegalArgumentException, InvocationTargetException,
            NoSuchMethodException, SecurityException, IOException {
        JsonTraverser t = new JsonTraverser();
        t.init(json);
        return (T) instantiateFromJson(t, c, context);
    }

    public static <T> T instantiateFromJson(String json, Class<T> c)
            throws InstantiationException, IllegalAccessException, IllegalArgumentException, InvocationTargetException,
            NoSuchMethodException, SecurityException, IOException {
        return instantiateFromJson(json, c, null);
    }

    public static <T> T instantiateFromJson(File json, Class<T> c, SerializationContext context)
            throws InstantiationException, IllegalAccessException, IllegalArgumentException, InvocationTargetException,
            NoSuchMethodException, SecurityException, IOException {
        JsonTraverser t = new JsonTraverser();
        t.init(json);
        return (T) instantiateFromJson(t, c, context);
    }

    public static <T> T instantiateFromJson(File json, Class<T> c)
            throws InstantiationException, IllegalAccessException, IllegalArgumentException, InvocationTargetException,
            NoSuchMethodException, SecurityException, IOException {
        return (T) instantiateFromJson(json, c, null);
    }

    public static <T> T instantiateFromJson(JsonTraverser t, Class<T> c, SerializationContext context)
            throws InstantiationException, IllegalAccessException, IllegalArgumentException, InvocationTargetException,
            NoSuchMethodException, SecurityException {
        TypeInfo inf = getTypeInfo(c);
        if (inf.isGeneric)
            throw new IllegalArgumentException(
                    "Trying to instantiate generic class" + c.getSimpleName() + " not from class field.");
        if (inf.subtyped) {
            ObjectIterable it = t.streamObject();
            if (!it.iterator().hasNext())
                t.expected(K_TYPE);
            Entry e = it.iterator().next();
            if (!e.key.equals(K_TYPE))
                t.expected(K_TYPE);
            String rType = t.getString().getRawString();
            if (!rType.equals(inf.type)) {
                Class<?> subC = inf.subtypes.get(rType);
                if (subC == null)
                    t.unexpected(e);
                return (T) registry.get(subC).instancer.instance(t, subC, it, context);
            }
        }
        return (T) inf.instancer.instance(t, c, null, context);
    }

    public static void fromJson(String json, Object o) throws InstantiationException, IllegalAccessException,
            IllegalArgumentException, InvocationTargetException, NoSuchMethodException, SecurityException, IOException {
        fromJson(json, o, null);
    }

    public static void fromJson(String json, Object o, SerializationContext context)
            throws InstantiationException, IllegalAccessException, IllegalArgumentException, InvocationTargetException,
            NoSuchMethodException, SecurityException, IOException {
        JsonTraverser t = new JsonTraverser();
        t.init(json);
        fromJson(t, o, context);
    }

    public static void fromJson(File json, Object o) throws InstantiationException, IllegalAccessException,
            IllegalArgumentException, InvocationTargetException, NoSuchMethodException, SecurityException, IOException {
        fromJson(json, o, null);
    }

    public static void fromJson(File json, Object o, SerializationContext context)
            throws InstantiationException, IllegalAccessException, IllegalArgumentException, InvocationTargetException,
            NoSuchMethodException, SecurityException, IOException {
        JsonTraverser t = new JsonTraverser();
        t.init(json);
        fromJson(t, o, context);
    }

    public static void fromJson(JsonTraverser t, Object o, SerializationContext context)
            throws IllegalAccessException, InstantiationException, InvocationTargetException, NoSuchMethodException {
        Class<?> c = o.getClass();
        TypeInfo inf = getTypeInfo(c);
        if (inf.reader == null)
            throw new IllegalArgumentException("Object type does not support in place json reading.");
        inf.reader.read(t, o, null, context);
    }

    static final HashMap<Class<?>, PrimitiveWriter> primitiveWriterRegistry;
    static final HashMap<Class<?>, PrimitiveReader> primitiveReaderRegistry;

    static void registerPrimitive(Class<?> c, PrimitiveWriter pw, PrimitiveReader pr) {
        primitiveWriterRegistry.put(c, pw);
        primitiveReaderRegistry.put(c, pr);
    }

    static class FieldInfo {
        final String name;
        final boolean primitive;
        final Class<?> c;
        final boolean isGeneric;
        final java.lang.reflect.Type[] genericC;
        final Field f;

        FieldInfo(Field f, JsonEntry je) throws IllegalArgumentException, IllegalAccessException {
            this.f = f;
            String n = je != null ? je.value() : null;
            if (n == null || n.length() == 0)
                n = f.getName();
            name = n;
            c = f.getType();
            primitive = c.isPrimitive();
            TypeInfo inf = getTypeInfo(c);
            isGeneric = inf.isGeneric;
            if (inf.isGeneric) {
                ParameterizedType pt = (ParameterizedType) f.getGenericType();
                genericC = pt.getActualTypeArguments();
            } else
                genericC = null;

        }
    }

    static class TypeInfo {
        Writer writer;
        Instancer instancer;
        Reader reader;
        boolean subtyped = false;
        HashMap<String, Class<?>> subtypes;
        String type;
        boolean isGeneric = false;
        GenericInstancer genericInstancer;
        GenericReader genericReader;

        private TypeInfo() {
        }

        public static TypeInfo newTypeInfo(Writer w, Instancer i) {
            TypeInfo inf = new TypeInfo();
            inf.writer = w;
            inf.instancer = i;
            return inf;
        }

        public static TypeInfo newGenericTypeInfo(Writer w, GenericInstancer i) {
            TypeInfo inf = new TypeInfo();
            inf.writer = w;
            inf.isGeneric = true;
            inf.genericInstancer = i;
            return inf;
        }

        public static TypeInfo newGenericTypeInfo(Writer w, GenericReader r) {
            TypeInfo inf = new TypeInfo();
            inf.writer = w;
            inf.isGeneric = true;
            inf.genericReader = r;
            return inf;
        }

        public static TypeInfo newSubtyped() {
            TypeInfo inf = new TypeInfo();
            inf.subtyped = true;
            inf.subtypes = new HashMap<>();
            inf.type = "";
            return inf;
        }

        public static TypeInfo newTypeInfo(Writer w, Instancer i, Reader r, String type) {
            TypeInfo inf = new TypeInfo();
            inf.writer = w;
            inf.instancer = i;
            inf.reader = r;
            inf.type = type;
            return inf;
        }
    }

    static final HashMap<Class<?>, TypeInfo> registry;

    private static TypeInfo getTypeInfo(Class<?> c) throws IllegalArgumentException, IllegalAccessException {
        TypeInfo inf = registry.get(c);
        if (inf != null)
            return inf;
        registerType(c);
        return registry.get(c);
    }

    static void register(Class<?> c, Writer w, Instancer i) {
        registry.put(c, TypeInfo.newTypeInfo(w, i));
    }

    static void registerGeneric(Class<?> c, Writer w, GenericInstancer i) {
        registry.put(c, TypeInfo.newGenericTypeInfo(w, i));
    }

    static void registerGeneric(Class<?> c, Writer w, GenericReader r) {
        registry.put(c, TypeInfo.newGenericTypeInfo(w, r));
    }

    public static void registerType(Class<?> c) throws IllegalArgumentException, IllegalAccessException {
        if (registry.containsKey(c))
            return;

        if (c.isInterface()) {
            registry.put(c, TypeInfo.newSubtyped());
            return;
        }

        Typed ta = c.getDeclaredAnnotation(Typed.class);
        JsonType jt = c.getDeclaredAnnotation(JsonType.class);
        FieldSelect fs = c.getDeclaredAnnotation(FieldSelect.class);

        boolean typed = ta != null;
        boolean obj = jt == null || jt.value() == Type.OBJECT;
        boolean isEnum = c.isEnum();
        boolean all = fs == null || fs.value() == Select.ALL;
        boolean abstr = Modifier.isAbstract(c.getModifiers());
        boolean custom = false;
        for (Class<?> i : c.getInterfaces())
            if (i.equals(CustomJson.class)) {
                custom = true;
                break;
            }
        // FieldOptional fo = ja.optional();

        if (typed && !obj)
            throw new IllegalArgumentException("Classes serialized as arrays cannot be typed.");

        if (typed && isEnum)
            throw new IllegalArgumentException("Enums cannot be typed.");

        String type = typed ? ta.value().length() > 0 ? ta.value() : c.getSimpleName() : null;

        Writer writer = null;
        Reader reader = null;
        Instancer instancer = null;

        if (isEnum) {
            HashMap<Object, String> names = new HashMap<>();
            HashMap<String, Object> variantMap = new HashMap<>();
            for (Field f : c.getDeclaredFields()) {
                if (f.isEnumConstant()) {
                    JsonEntry je = f.getDeclaredAnnotation(JsonEntry.class);
                    String name = f.getName();
                    if (je != null && je.value().length() > 0)
                        name = je.value();
                    Object val = f.get(null);
                    names.put(val, name);
                    variantMap.put(name, val);
                }
            }
            writer = (w, o, context) -> {
                String name = names.get(o);
                w.writeValue(name);
            };
            instancer = (t, cl, it, context) -> {
                String var = t.getString().getJsonString();
                Object res = variantMap.get(var);
                if (res == null)
                    throw new ParsingException("Invalid variant for enum " + cl.getSimpleName() + ": " + var);
                return res;
            };
        } else if (custom) {
            writer = (w, o, context) -> {
                ((CustomJson) o).write(w, context);
            };
            if (!abstr) {
                instancer = (t, cl, it, context) -> {
                    Object ob = cl.getDeclaredConstructor().newInstance();
                    ((CustomJson) ob).read(t, it, context);
                    return ob;
                };
                reader = (t, o, it, context) -> {
                    ((CustomJson) o).read(t, it, context);
                };
            }
        } else {

            final Vector<FieldInfo> fields = new Vector<>();
            final HashMap<String, FieldInfo> map = new HashMap<>();
            getFields(c, fields, map, obj, all);

            if (typed) {
                registerSubtype(c, type);
            }

            writer = (w, o, context) -> {
                if (obj)
                    w.startObject();
                else
                    w.startArray();

                if (typed) {
                    if (obj)
                        w.writeKey(K_TYPE);
                    w.writeValue(type);
                }

                for (FieldInfo f : fields) {
                    if (f.primitive) {
                        if (obj)
                            w.writeKey(f.name);
                        PrimitiveWriter pw = primitiveWriterRegistry.get(f.c);
                        if (pw == null)
                            throw new IllegalArgumentException("Missing primitive field type support for: " + f.name);
                        pw.write(w, o, f.f);
                    } else {
                        Object o2 = f.f.get(o);
                        if (o2 != null) {
                            if (obj)
                                w.writeKey(f.name);
                            Json.getTypeInfo(o2.getClass()).writer.write(w, o2, context);
                        }
                    }
                }

                if (obj)
                    w.endObject();
                else
                    w.endArray();
            };

            if (!abstr) {
                instancer = (t, cl, it, context) -> {
                    Object ob = cl.getDeclaredConstructor().newInstance();
                    if (obj) {
                        if (it == null) {
                            it = t.streamObject();
                            if (typed) { // Only check type if it was not already
                                if (!it.iterator().hasNext())
                                    t.expected(K_TYPE);
                                Entry e = it.iterator().next();
                                if (!e.key.getJsonString().equals(K_TYPE))
                                    t.expected(K_TYPE);
                                StringRef rType = t.getString();
                                if (!rType.equals(type))
                                    t.expectedStructureType(type, rType.getRawString());
                            }
                        }

                        for (Entry e : it) {
                            FieldInfo f = map.get(e.key.getJsonString());
                            if (f == null)
                                t.unexpected(e);

                            instanceField(t, ob, f, context);
                        }
                    } else {
                        ArrayIterable ita = t.streamArray();
                        for (FieldInfo f : fields) {
                            if (!ita.iterator().hasNext())
                                throw new ParsingException("Missing entries in " + c.getSimpleName() + " array");
                            ita.iterator().next();

                            instanceField(t, ob, f, context);
                        }

                        if (ita.iterator().hasNext())
                            throw new ParsingException("Too many entries in " + c.getSimpleName() + " array");
                    }
                    return ob;
                };

                reader = (t, o, it, context) -> {
                    if (obj) {
                        if (it == null) {
                            it = t.streamObject();
                            if (typed) { // Only check type if it was not already
                                if (!it.iterator().hasNext())
                                    t.expected(K_TYPE);
                                Entry e = it.iterator().next();
                                if (!e.key.getJsonString().equals(K_TYPE))
                                    t.expected(K_TYPE);
                                StringRef rType = t.getString();
                                if (!rType.equals(type))
                                    t.expectedStructureType(type, rType.getRawString());
                            }
                        }

                        for (Entry e : it) {
                            FieldInfo f = map.get(e.key.getJsonString());
                            if (f == null)
                                t.unexpected(e);
                            readField(t, o, f, context);
                        }
                    } else {
                        ArrayIterable ita = t.streamArray();
                        for (FieldInfo f : fields) {
                            if (!ita.iterator().hasNext())
                                throw new ParsingException("Missing entries in " + c.getSimpleName() + " array");
                            ita.iterator().next();
                            readField(t, o, f, context);
                        }

                        if (ita.iterator().hasNext())
                            throw new ParsingException("Too many entries in " + c.getSimpleName() + " array");
                    }
                };
            }
        }

        registry.put(c, TypeInfo.newTypeInfo(writer, instancer, reader, type));
    }

    static void instanceField(JsonTraverser t, Object ob, FieldInfo f, SerializationContext c)
            throws IllegalAccessException, IllegalArgumentException, InstantiationException, InvocationTargetException,
            NoSuchMethodException, SecurityException {
        if (f.primitive) {
            PrimitiveReader pr = primitiveReaderRegistry.get(f.c);
            if (pr == null)
                throw new IllegalArgumentException("Missing primitive field type support for: " + f.name);
            pr.read(t, ob, f.f);
            return;
        }
        if (f.isGeneric) {
            f.f.set(ob, getTypeInfo(f.c).genericInstancer.instance(t, f.genericC, c));
            return;
        }
        f.f.set(ob, instantiateFromJson(t, f.c, c));
    }

    static void readField(JsonTraverser t, Object o, FieldInfo f, SerializationContext c)
            throws IllegalAccessException, IllegalArgumentException, InstantiationException, InvocationTargetException,
            NoSuchMethodException, SecurityException {
        if (f.primitive) {
            PrimitiveReader pr = primitiveReaderRegistry.get(f.c);
            if (pr == null)
                throw new IllegalArgumentException("Missing primitive field type support for: " + f.name);
            pr.read(t, o, f.f);
            return;
        }
        TypeInfo inf = getTypeInfo(f.c);
        if (f.isGeneric) {
            if (inf.genericReader == null)
                f.f.set(o, inf.genericInstancer.instance(t, f.genericC, c));
            else
                inf.genericReader.read(t, f.f.get(o), null, f.genericC, c);
            return;
        }
        if (inf.subtyped) {
            TypeInfo subInf = getTypeInfo(f.f.get(o).getClass());
            ObjectIterable it = t.streamObject();
            if (!it.iterator().hasNext())
                t.expected(K_TYPE);
            Entry e = it.iterator().next();
            if (!e.key.equals(K_TYPE))
                t.expected(K_TYPE);
            String rType = t.getString().getRawString();
            if (!rType.equals(subInf.type)) {
                Class<?> subC = inf.subtypes.get(rType);
                if (subC == null)
                    t.unexpected(e);
                f.f.set(o, registry.get(subC).instancer.instance(t, subC, it, c));
            } else {
                if (subInf.reader == null)
                    f.f.set(o, subInf.instancer.instance(t, f.c, it, c));
                else
                    subInf.reader.read(t, f.f.get(o), it, c);
            }
        } else if (inf.reader == null)
            f.f.set(o, instantiateFromJson(t, f.c, c));
        else
            inf.reader.read(t, f.f.get(o), null, c);

    }

    static void getFields(Class<?> c, Vector<FieldInfo> fields, HashMap<String, FieldInfo> map, boolean obj,
            boolean all) throws IllegalArgumentException, IllegalAccessException {
        Class<?> superC = c.getSuperclass();
        if (superC != null && !superC.equals(Object.class)) {
            JsonType jt = superC.getDeclaredAnnotation(JsonType.class);
            boolean superObj = jt == null || jt.value() == Type.OBJECT;
            if (superObj != obj)
                throw new IllegalArgumentException("Json Subclass has different structure type than superclass. "
                        + "Subclass " + c.getSimpleName() + "has type: " + (obj ? "object" : "array") + ", super type "
                        + superC.getSimpleName() + " has type: " + (obj ? "object" : "array"));
            FieldSelect fs = superC.getDeclaredAnnotation(FieldSelect.class);
            boolean superAll = fs == null || fs.value() == Select.ALL;

            getFields(superC, fields, map, obj, superAll);
        }

        for (Field f : c.getDeclaredFields()) {
            if (f.isSynthetic())
                continue;
            int mod = f.getModifiers();
            if (Modifier.isStatic(mod) || Modifier.isTransient(mod))
                continue;
            f.setAccessible(true);
            JsonEntry je = f.getAnnotation(JsonEntry.class);
            if (je == null && !all)
                continue;

            FieldInfo inf = new FieldInfo(f, je);
            fields.add(inf);
            map.put(inf.name, inf);
        }
    }

    static void registerSubtype(Class<?> baseClass, String baseType)
            throws IllegalArgumentException, IllegalAccessException {
        registerSubtype(baseClass, baseType, baseClass);
    }

    static void registerSubtype(Class<?> baseClass, String baseType, Class<?> c)
            throws IllegalArgumentException, IllegalAccessException {

        Class<?>[] interfaces = c.getInterfaces();
        for (Class<?> i : interfaces) {
            TypeInfo iinf = getTypeInfo(i);
            if (iinf.subtyped == false) {
                iinf.subtyped = true;
                iinf.subtypes = new HashMap<>();
            }
            iinf.subtypes.put(baseType, baseClass);
        }

        Class<?> superC = c.getSuperclass();
        if (superC == null || superC.equals(Object.class))
            return;

        boolean abstr = Modifier.isAbstract(superC.getModifiers());
        if (!abstr && superC.getAnnotation(Typed.class) == null)
            throw new IllegalArgumentException("Super-class " + superC.getSimpleName() + " of subtyped class "
                    + baseClass.getSimpleName() + " is not @Typed");
        TypeInfo inf = getTypeInfo(superC);
        if (inf.subtyped == false) {
            inf.subtyped = true;
            inf.subtypes = new HashMap<>();
        }
        inf.subtypes.put(baseType, baseClass);

        registerSubtype(baseClass, baseType, superC);
    }

    static public interface Writer {
        void write(JsonWriter w, Object o, SerializationContext context) throws IllegalAccessException;
    }

    static public interface Instancer {
        Object instance(JsonTraverser t, Class<?> c, ObjectIterable it, SerializationContext context)
                throws IllegalAccessException, InstantiationException, InvocationTargetException, NoSuchMethodException;
    }

    static public interface Reader {
        void read(JsonTraverser t, Object o, ObjectIterable it, SerializationContext context)
                throws IllegalAccessException, InstantiationException, InvocationTargetException, NoSuchMethodException;
    }

    static interface PrimitiveWriter {
        void write(JsonWriter w, Object o, Field f) throws IllegalAccessException;
    }

    static interface PrimitiveReader {
        void read(JsonTraverser t, Object o, Field f) throws IllegalAccessException;
    }

    static interface GenericInstancer {
        Object instance(JsonTraverser t, java.lang.reflect.Type[] generics, SerializationContext context)
                throws IllegalAccessException, InstantiationException, InvocationTargetException, NoSuchMethodException;
    }

    static interface GenericReader {
        void read(JsonTraverser t, Object o, ObjectIterable it, java.lang.reflect.Type[] generics,
                SerializationContext context)
                throws IllegalAccessException, InstantiationException, InvocationTargetException, NoSuchMethodException;
    }

    static {
        registry = new HashMap<>();
        primitiveWriterRegistry = new HashMap<>();
        primitiveReaderRegistry = new HashMap<>();

        registerPrimitive(double.class, (w, o, f) -> w.writeValue(f.getDouble(o)),
                (t, o, f) -> f.setDouble(o, t.getDouble()));
        registerPrimitive(int.class, (w, o, f) -> w.writeValue(f.getInt(o)),
                (t, o, f) -> f.setInt(o, (int) t.getLong()));
        registerPrimitive(float.class, (w, o, f) -> w.writeValue(f.getFloat(o)),
                (t, o, f) -> f.setFloat(o, (float) t.getDouble()));
        registerPrimitive(char.class, (w, o, f) -> w.writeValue(f.getChar(o)),
                (t, o, f) -> f.setChar(o, (char) t.getLong()));
        registerPrimitive(byte.class, (w, o, f) -> w.writeValue(f.getByte(o)),
                (t, o, f) -> f.setByte(o, (byte) t.getLong()));
        registerPrimitive(long.class, (w, o, f) -> w.writeValue(f.getLong(o)), (t, o, f) -> f.setLong(o, t.getLong()));
        registerPrimitive(short.class, (w, o, f) -> w.writeValue(f.getShort(o)),
                (t, o, f) -> f.setShort(o, (short) t.getLong()));
        registerPrimitive(boolean.class, (w, o, f) -> w.writeValue(f.getBoolean(o)),
                (t, o, f) -> f.setBoolean(o, t.getBoolean()));
        registerPrimitive(Optional.class, (w, o, f) -> w.writeValue(f.getBoolean(o)),
                (t, o, f) -> f.setBoolean(o, t.getBoolean()));

        register(String.class, (j, o, c) -> j.writeValue((String) o), (j, o, i, c) -> j.getString().getJsonString());
        register(Double.class, (j, o, c) -> j.writeValue((Double) o), (j, o, i, c) -> j.getDouble());
        register(Float.class, (j, o, c) -> j.writeValue((Float) o), (j, o, i, c) -> (float) j.getDouble());
        register(Long.class, (j, o, c) -> j.writeValue((Long) o), (j, o, i, c) -> j.getLong());
        register(Integer.class, (j, o, c) -> j.writeValue((Integer) o), (j, o, i, c) -> (int) j.getLong());
        register(Short.class, (j, o, c) -> j.writeValue((Short) o), (j, o, i, c) -> (short) j.getLong());
        register(Byte.class, (j, o, c) -> j.writeValue((Byte) o), (j, o, i, c) -> (byte) j.getLong());
        register(Character.class, (j, o, c) -> j.writeValue((Character) o), (j, o, i, c) -> (char) j.getLong());
        register(Boolean.class, (j, o, c) -> j.writeValue((Boolean) o), (j, o, i, c) -> j.getBoolean());
        register(double[].class, (j, o, c) -> {
            j.startArray();
            double[] arr = (double[]) o;
            j.writeValue(arr.length);
            for (double v : arr)
                j.writeValue(v);
            j.endArray();
        }, (j, o, a, c) -> {
            ArrayIterable it = j.streamArray();
            if (!it.iterator().hasNext())
                throw new ParsingException("Expected array length at start of array");
            it.iterator().next();
            int length = (int) j.getLong(), i = 0;
            double[] arr = new double[length];
            for (ValueType ty : it)
                arr[i++] = j.getDouble();
            return arr;
        });
        register(float[].class, (j, o, c) -> {
            j.startArray();
            float[] arr = (float[]) o;
            j.writeValue(arr.length);
            for (float v : arr)
                j.writeValue(v);
            j.endArray();
        }, (j, o, a, c) -> {
            ArrayIterable it = j.streamArray();
            if (!it.iterator().hasNext())
                throw new ParsingException("Expected array length at start of array");
            it.iterator().next();
            int length = (int) j.getLong(), i = 0;
            float[] arr = new float[length];
            for (ValueType ty : it)
                arr[i++] = (float) j.getDouble();
            return arr;
        });
        register(long[].class, (j, o, c) -> {
            j.startArray();
            long[] arr = (long[]) o;
            j.writeValue(arr.length);
            for (long v : arr)
                j.writeValue(v);
            j.endArray();
        }, (j, o, a, c) -> {
            ArrayIterable it = j.streamArray();
            if (!it.iterator().hasNext())
                throw new ParsingException("Expected array length at start of array");
            it.iterator().next();
            int length = (int) j.getLong(), i = 0;
            long[] arr = new long[length];
            for (ValueType ty : it)
                arr[i++] = j.getLong();
            return arr;
        });
        register(int[].class, (j, o, c) -> {
            j.startArray();
            int[] arr = (int[]) o;
            j.writeValue(arr.length);
            for (int v : arr)
                j.writeValue(v);
            j.endArray();
        }, (j, o, a, c) -> {
            ArrayIterable it = j.streamArray();
            if (!it.iterator().hasNext())
                throw new ParsingException("Expected array length at start of array");
            it.iterator().next();
            int length = (int) j.getLong(), i = 0;
            int[] arr = new int[length];
            for (ValueType ty : it)
                arr[i++] = (int) j.getLong();
            return arr;
        });
        register(short[].class, (j, o, c) -> {
            j.startArray();
            short[] arr = (short[]) o;
            j.writeValue(arr.length);
            for (short v : arr)
                j.writeValue(v);
            j.endArray();
        }, (j, o, a, c) -> {
            ArrayIterable it = j.streamArray();
            if (!it.iterator().hasNext())
                throw new ParsingException("Expected array length at start of array");
            it.iterator().next();
            int length = (int) j.getLong(), i = 0;
            short[] arr = new short[length];
            for (ValueType ty : it)
                arr[i++] = (short) j.getLong();
            return arr;
        });
        register(byte[].class, (j, o, c) -> {
            j.startArray();
            byte[] arr = (byte[]) o;
            j.writeValue(arr.length);
            for (short v : arr)
                j.writeValue(v);
            j.endArray();
        }, (j, o, a, c) -> {
            ArrayIterable it = j.streamArray();
            if (!it.iterator().hasNext())
                throw new ParsingException("Expected array length at start of array");
            it.iterator().next();
            int length = (int) j.getLong(), i = 0;
            byte[] arr = new byte[length];
            for (ValueType ty : it)
                arr[i++] = (byte) j.getLong();
            return arr;
        });
        register(char[].class, (j, o, c) -> {
            j.startArray();
            char[] arr = (char[]) o;
            j.writeValue(arr.length);
            for (char v : arr)
                j.writeValue(v);
            j.endArray();
        }, (j, o, a, c) -> {
            ArrayIterable it = j.streamArray();
            if (!it.iterator().hasNext())
                throw new ParsingException("Expected array length at start of array");
            it.iterator().next();
            int length = (int) j.getLong(), i = 0;
            char[] arr = new char[length];
            for (ValueType ty : it)
                arr[i++] = (char) j.getLong();
            return arr;
        });
        register(boolean[].class, (j, o, c) -> {
            j.startArray();
            boolean[] arr = (boolean[]) o;
            j.writeValue(arr.length);
            for (boolean v : arr)
                j.writeValue(v);
            j.endArray();
        }, (j, o, a, c) -> {
            ArrayIterable it = j.streamArray();
            if (!it.iterator().hasNext())
                throw new ParsingException("Expected array length at start of array");
            it.iterator().next();
            int length = (int) j.getLong(), i = 0;
            boolean[] arr = new boolean[length];
            for (ValueType ty : it)
                arr[i++] = j.getBoolean();
            return arr;
        });
        register(Duration.class, (j, o, c) -> {
            j.startArray();
            Duration d = (Duration) o;
            j.writeValue(d.getSeconds());
            j.writeValue(d.getNano());
            j.endArray();
        }, (j, o, a, c) -> {
            Iterator<ValueType> it = j.streamArray().iterator();
            if (!it.hasNext())
                j.expected("seconds");
            it.next();
            long sec = j.getLong();
            if (!it.hasNext())
                j.expected("nanosecs");
            it.next();
            long nano = j.getLong();
            if (it.hasNext())
                j.expected("nothing");
            return Duration.ofSeconds(sec, nano);
        });
        register(Instant.class, (j, o, c) -> {
            j.startArray();
            Instant d = (Instant) o;
            j.writeValue(d.getEpochSecond());
            j.writeValue(d.getNano());
            j.endArray();
        }, (j, o, a, c) -> {
            Iterator<ValueType> it = j.streamArray().iterator();
            if (!it.hasNext())
                j.expected("seconds");
            it.next();
            long sec = j.getLong();
            if (!it.hasNext())
                j.expected("nanosecs");
            it.next();
            long nano = j.getLong();
            if (it.hasNext())
                j.expected("nothing");
            return Instant.ofEpochSecond(sec, nano);
        });
        registerGeneric(Vector.class, (j, o, c) -> {
            j.startArray();
            Vector<?> vec = (Vector<?>) o;
            for (Object v : vec) {
                Json.toJson(j, v, c);
            }
            j.endArray();
        }, (t, gc, c) -> {
            Vector<Object> v = new Vector<>();
            for (ValueType vt : t.streamArray()) {
                v.add(Json.instantiateFromJson(t, (Class<?>) gc[0], c));
            }
            return v;
        });
        registerGeneric(Optional.class, (j, o, c) -> {
            Optional<Object> opt = (Optional<Object>) o;
            if (opt.isPresent()) {
                Json.toJson(j, opt.get(), c);
            } else {
                j.startObject();
                j.endObject();
            }
        }, (t, gc, c) -> {
            if (t.isEmpty())
                return Optional.empty();
            return Optional.of(Json.instantiateFromJson(t, (Class<?>) gc[0], c));
        });
        registerGeneric(HashMap.class, (j, o, c) -> {
            HashMap<String, Object> map = (HashMap<String, Object>) o;
            Iterator it = map.entrySet().iterator();
            j.startObject();
            while (it.hasNext()) {
                HashMap.Entry<String, Object> pair = (HashMap.Entry) it.next();
                j.writeKey(pair.getKey());
                Json.toJson(j, pair.getValue(), c);
                // it.remove(); // avoids a ConcurrentModificationException
            }
            j.endObject();
        }, (t, o, it, generics, context) -> {
            HashMap<String, Object> map = (HashMap<String, Object>) o;
            if (it == null)
                it = t.streamObject();
            for (Entry e : it) {
                String key = e.key.getJsonString();
                Object o2 = map.get(key);
                if (o2 == null)
                    map.put(key, Json.instantiateFromJson(t, (Class<?>) generics[1], context));
                else
                    Json.fromJson(t, o2, context);
            }
        });
        registerGeneric(Stack.class, (j, o, c) -> {
                Stack stack = (Stack) o;
                j.startArray();
                Iterator it = stack.iterator();
                while (it.hasNext()) {
                    Json.toJson(j, it.next(), c);
                }
                j.endArray();
            },
            (t, o, it, generics, context) -> {
                Stack stack = (Stack)o;
                for (ValueType vt : t.streamArray()){
                    stack.add(Json.instantiateFromJson(t, (Class<?>)generics[0], context));
                }
            }
        );
    }
}