/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.utils.json;

import java.util.Vector;

import org.junit.Assert;
import org.junit.Test;

import de.rwth.montisim.commons.utils.BuildContext;
import de.rwth.montisim.commons.utils.json.JsonTraverser.Entry;
import de.rwth.montisim.commons.utils.json.JsonTraverser.ObjectIterable;

public class AnnotatedJsonTest {

    @Test
    public void simpleTest1() throws SerializationException {
        SimpleClass1 o = new SimpleClass1();

        String res = Json.toFormattedJson(o);
        // System.out.println(res);
        Assert.assertEquals("{\n" + "  \"s1\": \"a string\",\n" + "  \"double1\": 5.6\n" + "}", res);
    }

    @Test
    public void simpleTest2() throws SerializationException {
        SimpleClass2 o = new SimpleClass2();

        String res = Json.toFormattedJson(o);
        // System.out.println(res);
        Assert.assertEquals("{\n" + "  \"s1\": \"abcde\",\n" + "  \"i1\": 4,\n" + "  \"d1\": 5.6,\n"
                + "  \"aaa\": 4.2,\n" + "  \"c1\": 65,\n" + "  \"sh1\": 10,\n" + "  \"l1\": -5,\n" + "  \"b1\": 8,\n"
                + "  \"z1\": true\n" + "}", res);

    }

    @Test
    public void emptyTest1() throws SerializationException {
        EmptyClass1 o = new EmptyClass1();

        String res = Json.toFormattedJson(o);
        // System.out.println(res);
        Assert.assertEquals("{}", res);

    }

    @Test
    public void simpleTest3() throws SerializationException {
        SimpleClass3 o = new SimpleClass3();

        String res = Json.toFormattedJson(o);
        // System.out.println(res);
        Assert.assertEquals("[\n" + "  \"text\",\n" + "  1,\n" + "  100.0\n" + "]", res);

    }

    @Test
    public void nestedTest1() throws SerializationException {
        NestedClass1 o = new NestedClass1();

        String res = Json.toFormattedJson(o);
        // System.out.println(res);
        Assert.assertEquals("{\n" + "  \"s1\": \"abcde\",\n" + "  \"c1\": {\n" + "    \"s1\": \"a string\",\n"
                + "    \"double1\": 5.6\n" + "  },\n" + "  \"nested_entry\": {\n" + "    \"s1\": \"abcde\",\n"
                + "    \"i1\": 4,\n" + "    \"d1\": 5.6,\n" + "    \"aaa\": 4.2,\n" + "    \"c1\": 65,\n"
                + "    \"sh1\": 10,\n" + "    \"l1\": -5,\n" + "    \"b1\": 8,\n" + "    \"z1\": true\n" + "  },\n"
                + "  \"e1\": {},\n" + "  \"arraySubClass\": [\n" + "    \"text\",\n" + "    1,\n" + "    100.0\n"
                + "  ],\n" + "  \"z1\": true,\n" + "  \"i1\": {\n" + "    \"type\": \"ISubclass1\",\n"
                + "    \"f1\": -1,\n" + "    \"f2\": -2.0\n" + "  }\n" + "}", res);

    }

    @Test
    public void typedTest1() throws SerializationException {
        TypedClass1 o = new TypedClass1();

        String res = Json.toFormattedJson(o);
        // System.out.println(res);
        Assert.assertEquals("{\n" + "  \"type\": \"TypedClass1\",\n" + "  \"a\": 1,\n" + "  \"b\": 2\n" + "}", res);
    }

    @Test
    public void typedTest2() throws SerializationException {
        TypedClass2 o = new TypedClass2();

        String res = Json.toFormattedJson(o);
        // System.out.println(res);
        Assert.assertEquals("{\n" + "  \"type\": \"type2\",\n" + "  \"a\": 1,\n" + "  \"b\": 2\n" + "}", res);
    }

    @Test
    public void subclassTest1() throws SerializationException {
        Subclass1 o = new Subclass1();

        String res = Json.toFormattedJson(o);
        // System.out.println(res);
        Assert.assertEquals(
                "{\n" + "  \"s1\": \"a string\",\n" + "  \"double1\": 5.6,\n" + "  \"a\": 1,\n" + "  \"b\": 2\n" + "}",
                res);
    }

    @Test
    public void subclassTest2() throws SerializationException {
        Subclass2 o = new Subclass2();

        String res = Json.toFormattedJson(o);
        // System.out.println(res);
        Assert.assertEquals("{\n" + "  \"type\": \"sub2\",\n" + "  \"s1\": \"a string\",\n" + "  \"double1\": 5.6,\n"
                + "  \"a\": 1,\n" + "  \"b\": 2\n" + "}", res);
    }

    @Test
    public void enumTest1() throws SerializationException {
        EnumClass1 o = new EnumClass1();

        String res = Json.toFormattedJson(o);
        // System.out.println(res);
        Assert.assertEquals("{\n" + "  \"f1\": \"VARIANT1\",\n" + "  \"f2\": \"variant_2\"\n" + "}", res);
    }

    @Test
    public void instantiateTest1() throws SerializationException {
        SimpleClass1 o = new SimpleClass1();
        o.set2();

        // System.out.println("Constructors");
        // for (Constructor c : SimpleClass1.class.getDeclaredConstructors()){
        // System.out.println(c);
        // }

        String res = Json.toFormattedJson(o);
        // System.out.println(res);
        SimpleClass1 o2 = Json.instantiateFromJson(res, SimpleClass1.class);
        o.assertEquals(o2);
    }

    @Test
    public void instantiateTest2() throws SerializationException {
        SimpleClass2 o = new SimpleClass2();
        o.set2();

        String res = Json.toFormattedJson(o);
        // System.out.println(res);
        SimpleClass2 o2 = Json.instantiateFromJson(res, SimpleClass2.class);
        o.assertEquals(o2);
    }

    @Test
    public void instantiateTest3() throws SerializationException {
        SimpleClass3 o = new SimpleClass3();
        o.set2();

        // System.out.println("Fields");
        // for (Field f : SimpleClass3.class.getDeclaredFields()){
        // System.out.println(f);
        // }

        String res = Json.toFormattedJson(o);
        // System.out.println(res);
        SimpleClass3 o2 = Json.instantiateFromJson(res, SimpleClass3.class);
        o.assertEquals(o2);
    }

    @Test
    public void instantiateNestedTest() throws SerializationException {
        NestedClass1 o = new NestedClass1();
        o.set2();

        String res = Json.toFormattedJson(o);
        // System.out.println(res);
        NestedClass1 o2 = Json.instantiateFromJson(res, NestedClass1.class);
        o.assertEquals(o2);
    }

    @Test
    public void instantiateTyped1Test() throws SerializationException {
        TypedClass1 o = new TypedClass1();
        o.set2();

        String res = Json.toFormattedJson(o);
        // System.out.println(res);
        TypedClass1 o2 = Json.instantiateFromJson(res, TypedClass1.class);
        o.assertEquals(o2);
    }

    @Test
    public void instantiateTyped2Test() throws SerializationException {
        TypedClass2 o = new TypedClass2();
        o.set2();

        String res = Json.toFormattedJson(o);
        // System.out.println(res);
        TypedClass2 o2 = Json.instantiateFromJson(res, TypedClass2.class);
        o.assertEquals(o2);
    }

    @Test
    public void instantiateSubclass1Test() throws SerializationException {
        Subclass1 o = new Subclass1();
        o.sset2();

        String res = Json.toFormattedJson(o);
        // System.out.println(res);
        Subclass1 o2 = Json.instantiateFromJson(res, Subclass1.class);
        o.sassertEquals(o2);
    }

    @Test
    public void instantiateSubclass2Test() throws SerializationException {
        Subclass2 o = new Subclass2();
        o.sset2();

        String res = Json.toFormattedJson(o);
        // System.out.println(res);
        Subclass2 o2 = Json.instantiateFromJson(res, Subclass2.class);
        o.sassertEquals(o2);
    }

    @Test
    public void instantiateSubtyped1Test() throws SerializationException {
        NestedClass2 o = new NestedClass2();
        o.set2();

        String res = Json.toFormattedJson(o);
        // System.out.println(res);
        NestedClass2 o2 = Json.instantiateFromJson(res, NestedClass2.class);
        o.assertEquals(o2);
    }

    @Test
    public void instantiateSubtyped2Test() throws SerializationException {
        Subclass2 o = new Subclass2();
        o.set2();

        String res = Json.toFormattedJson(o);
        // System.out.println(res);
        SuperClass1 o2 = Json.instantiateFromJson(res, SuperClass1.class);
        Assert.assertTrue(o2 instanceof Subclass2);
        o.sassertEquals((Subclass2) o2);
    }

    @Test
    public void instantiateSubtypedInterface1Test() throws SerializationException {
        ISubclass1 o = new ISubclass1();
        o.set2();

        String res = Json.toFormattedJson(o);
        // System.out.println(res);
        Interface1 o2 = Json.instantiateFromJson(res, Interface1.class);
        Assert.assertTrue(o2 instanceof ISubclass1);
        o.assertEquals((ISubclass1) o2);
    }

    @Test
    public void testFromJson1() throws SerializationException {
        SimpleClass1 o = new SimpleClass1();
        Json.fromJson("{\"s1\": \"other string\"}", o);

        SimpleClass1 o2 = new SimpleClass1();
        o2.set3();
        o2.assertEquals(o);
    }

    @Test
    public void testFromJson2() throws SerializationException {
        NestedClass1 o = new NestedClass1();
        Json.fromJson("{\"s1\": \"xxxxxx\", \"c1\":{\"s1\": \"other string\"}, \"z1\": false }", o);

        NestedClass1 o2 = new NestedClass1();
        o2.set3();
        o2.assertEquals(o);
    }

    @Test
    public void testFromJson3() throws SerializationException {

        NestedClass1 o = new NestedClass1();
        o.set4();
        Json.fromJson("{\"i1\":{\"type\":\"ISubclass1\", \"f1\": 42}}", o);

        NestedClass1 o2 = new NestedClass1();
        o2.set5();
        o2.assertEquals(o);
    }

    @Test
    public void testFromJson4() throws SerializationException {

        NestedClass1 o = new NestedClass1();
        o.set2();
        Json.fromJson("{\"i1\":{\"type\":\"ISubclass1\", \"f1\": 42, \"f2\": -7}}", o);

        NestedClass1 o2 = new NestedClass1();
        o2.set2();
        o2.set6();
        o2.assertEquals(o);
    }

    @Test
    public void customTest1() throws SerializationException {

        CustomClass1 o = new CustomClass1();
        o.set2();

        String res = Json.toFormattedJson(o);
        // System.out.println(res);
        CustomClass1 o2 = Json.instantiateFromJson(res, CustomClass1.class);
        o.assertEquals(o2);
    }

    @Test
    public void vectorTest1() throws SerializationException {

        VectorClass1 o = new VectorClass1();
        o.set2();

        // Class<?> c = o.getClass();
        // System.out.println("VectorClass1 class: "+c.toString());
        // Field f = c.getDeclaredField("vec");
        // System.out.println("Field vec: "+f.toGenericString());
        // ParameterizedType pt = (ParameterizedType) f.getGenericType();
        // System.out.println("ParamType: "+pt.toString());
        // Class<?> genC = (Class<?>) pt.getActualTypeArguments()[0];
        // System.out.println("actualType[0]: "+genC);

        String res = Json.toFormattedJson(o);
        // System.out.println(res);
        VectorClass1 o2 = Json.instantiateFromJson(res, VectorClass1.class);
        o.assertEquals(o2);
    }

    @Test
    public void instantiateEnum1Test() throws SerializationException {
        EnumClass1 o = new EnumClass1();
        o.set2();

        String res = Json.toFormattedJson(o);
        // System.out.println(res);
        EnumClass1 o2 = Json.instantiateFromJson(res, EnumClass1.class);
        o.assertEquals(o2);
    }

    @Test
    public void instantiateEnum2Test() throws SerializationException {
        EnumClass1 o = new EnumClass1();
        o.set2();

        String res = Json.toFormattedJson(o);
        // System.out.println(res);
        EnumClass1 o2 = Json.instantiateFromJson(res, EnumClass1.class);
        o.assertEquals(o2);
    }
}

@FieldSelect(Select.EXPLICIT)
class SimpleClass1 {
    @JsonEntry
    String s1;
    int i1; // Is NOT serialized
    @JsonEntry("double1")
    double d1;

    SimpleClass1() {
        set1();
    }

    void set1() {
        s1 = "a string";
        i1 = 4;
        d1 = 5.6;
    }

    void set2() {
        s1 = "doubidou";
        i1 = 4;
        d1 = 1.1;
    }

    void set3() {
        s1 = "other string";
    }

    public void assertEquals(SimpleClass1 o2) {
        Assert.assertEquals("s1", s1, o2.s1);
        Assert.assertEquals("d1", d1, o2.d1, 0.00001);
    }
}

class SimpleClass2 {
    String s1;
    int i1;
    double d1;
    @JsonEntry("aaa")
    float f1;
    char c1;
    // @IgnoreField
    transient String s2 = "chocolate";
    short sh1;
    long l1;
    byte b1;
    boolean z1;

    SimpleClass2() {
        set1();
    }

    void set1() {
        s1 = "abcde";
        i1 = 4;
        d1 = 5.6;
        f1 = 4.2f;
        c1 = 'A';
        sh1 = 10;
        l1 = -5;
        b1 = 8;
        z1 = true;
    }

    void set2() {
        s1 = "zzzz";
        i1 = 480;
        d1 = -8.8;
        f1 = -7.4f;
        c1 = '@';
        sh1 = -150;
        l1 = 55555;
        b1 = -7;
        z1 = false;
    }

    public void assertEquals(SimpleClass2 o2) {
        Assert.assertEquals("s1", s1, o2.s1);
        Assert.assertEquals("i1", i1, o2.i1);
        Assert.assertEquals("d1", d1, o2.d1, 0.00001);
        Assert.assertEquals("f1", f1, o2.f1, 0.00001f);
        Assert.assertEquals("c1", c1, o2.c1);
        Assert.assertEquals("sh1", sh1, o2.sh1);
        Assert.assertEquals("l1", l1, o2.l1);
        Assert.assertEquals("b1", b1, o2.b1);
        Assert.assertEquals("z1", z1, o2.z1);
    }
}

@FieldSelect(Select.EXPLICIT)
class EmptyClass1 {
    String s1;
    int i1;
    double d1;
}

@JsonType(Type.ARRAY)
class SimpleClass3 {
    String s1;
    private int i1;
    double d1;

    SimpleClass3() {
        set1();
    }

    void set1() {
        s1 = "text";
        i1 = 1;
        d1 = 100;
    }

    void set2() {
        s1 = "other text";
        i1 = 78;
        d1 = -100.1;
    }

    public void assertEquals(SimpleClass3 o2) {
        Assert.assertEquals("s1", s1, o2.s1);
        Assert.assertEquals("d1", d1, o2.d1, 0.00001);
    }
}

class NestedClass1 {
    String s1;
    SimpleClass1 c1;
    @JsonEntry("nested_entry")
    SimpleClass2 c2;
    // @IgnoreField
    transient SimpleClass2 c3;
    EmptyClass1 e1;
    SimpleClass3 arraySubClass;
    boolean z1;
    Interface1 i1;

    NestedClass1() {
        set1();
    }

    void set1() {
        s1 = "abcde";
        c1 = new SimpleClass1();
        c2 = new SimpleClass2();
        e1 = new EmptyClass1();
        arraySubClass = new SimpleClass3();
        z1 = true;
        i1 = new ISubclass1();
    }

    void set2() {
        s1 = "edcba";
        c1 = new SimpleClass1();
        c1.set2();
        c2 = new SimpleClass2();
        c2.set2();
        e1 = new EmptyClass1();
        arraySubClass = new SimpleClass3();
        arraySubClass.set2();
        z1 = false;
        i1 = new ISubclass2();
    }

    void set3() {
        s1 = "xxxxxx";
        c1.set3();
        z1 = false;
    }

    void set4() {
        ISubclass1 i = new ISubclass1();
        i1 = i;
        i.set2();
    }

    void set5() {
        ISubclass1 i = new ISubclass1();
        i1 = i;
        i.set2();
        i.set3();
    }

    void set6() {
        ISubclass1 i = new ISubclass1();
        i1 = i;
        i.f1 = 42;
        i.f2 = -7;
    }

    public void assertEquals(NestedClass1 o2) {
        Assert.assertEquals("s1", s1, o2.s1);
        c1.assertEquals(o2.c1);
        c2.assertEquals(o2.c2);
        arraySubClass.assertEquals(o2.arraySubClass);
        Assert.assertEquals("z1", z1, o2.z1);
        Assert.assertTrue(i1.getClass().equals(o2.i1.getClass()));
        if (i1 instanceof ISubclass1) {
            ((ISubclass1) i1).assertEquals((ISubclass1) o2.i1);
        } else {
            ((ISubclass2) i1).assertEquals((ISubclass2) o2.i1);
        }
    }
}

@Typed
class TypedClass1 {
    int a;
    int b;

    TypedClass1() {
        set1();
    }

    void set1() {
        a = 1;
        b = 2;
    }

    void set2() {
        a = 3;
        b = 4;
    }

    public void assertEquals(TypedClass1 o2) {
        Assert.assertEquals("a", a, o2.a);
        Assert.assertEquals("b", b, o2.b);
    }
}

@Typed("type2")
class TypedClass2 {
    int a;
    int b;

    TypedClass2() {
        set1();
    }

    void set1() {
        a = 1;
        b = 2;
    }

    void set2() {
        a = 3;
        b = 4;
    }

    public void assertEquals(TypedClass2 o2) {
        Assert.assertEquals("a", a, o2.a);
        Assert.assertEquals("b", b, o2.b);
    }
}

class Subclass1 extends SimpleClass1 {
    int a;
    int b;

    Subclass1() {
        sset1();
    }

    void sset1() {
        set1();
        a = 1;
        b = 2;
    }

    void sset2() {
        set2();
        a = 3;
        b = 4;
    }

    public void sassertEquals(Subclass1 o2) {
        assertEquals(o2);
        Assert.assertEquals("a", a, o2.a);
        Assert.assertEquals("b", b, o2.b);
    }
}

@FieldSelect(Select.EXPLICIT)
@Typed
class SuperClass1 {
    @JsonEntry
    String s1;
    int i1; // Is NOT serialized
    @JsonEntry("double1")
    double d1;

    SuperClass1() {
        set1();
    }

    void set1() {
        s1 = "a string";
        i1 = 4;
        d1 = 5.6;
    }

    void set2() {
        s1 = "doubidou";
        i1 = 4;
        d1 = 1.1;
    }

    public void assertEquals(SuperClass1 o2) {
        Assert.assertEquals("s1", s1, o2.s1);
        Assert.assertEquals("d1", d1, o2.d1, 0.00001);
    }
}

@Typed("sub2")
class Subclass2 extends SuperClass1 {
    int a;
    int b;

    Subclass2() {
        sset1();
    }

    void sset1() {
        set1();
        a = 1;
        b = 2;
    }

    void sset2() {
        set2();
        a = 3;
        b = 4;
    }

    public void sassertEquals(Subclass2 o2) {
        assertEquals(o2);
        Assert.assertEquals("a", a, o2.a);
        Assert.assertEquals("b", b, o2.b);
    }
}

@Typed()
class Subclass3 extends SuperClass1 {
    static {
        Json.registerType(Subclass3.class);
    }
    int c;
    int d;

    Subclass3() {
        sset1();
    }

    void sset1() {
        set1();
        c = -1;
        d = -2;
    }

    void sset2() {
        set2();
        c = -3;
        d = -4;
    }

    public void sassertEquals(Subclass3 o2) {
        assertEquals(o2);
        Assert.assertEquals("c", c, o2.c);
        Assert.assertEquals("d", d, o2.d);
    }
}

@Typed()
class NestedClass2 {
    SuperClass1 c;

    NestedClass2() {
        set1();
    }

    void set1() {
        c = new Subclass2();
    }

    void set2() {
        c = new Subclass3();
    }

    public void assertEquals(NestedClass2 o2) {
        Assert.assertEquals("Missmatching classes", c.getClass(), o2.c.getClass());
        if (c instanceof Subclass2) {
            ((Subclass2) c).sassertEquals((Subclass2) o2.c);
        } else if (c instanceof Subclass3) {
            ((Subclass3) c).sassertEquals((Subclass3) o2.c);
        } else {
            throw new IllegalArgumentException("Unknown sub-instance");
        }
    }
}

interface Interface1 {
    void someFunc();
}

@Typed
class ISubclass1 implements Interface1 {
    static {
        Json.registerType(ISubclass1.class);
    }
    int f1;
    double f2;

    @Override
    public void someFunc() {
    }

    ISubclass1() {
        set1();
    }

    void set1() {
        f1 = -1;
        f2 = -2;
    }

    void set2() {
        f1 = -3;
        f2 = -4;
    }

    void set3() {
        f1 = 42;
    }

    public void assertEquals(ISubclass1 o2) {
        Assert.assertEquals("f1", f1, o2.f1);
        Assert.assertEquals("f2", f2, o2.f2, 0.00001);
    }
}

@Typed
class ISubclass2 implements Interface1 {
    static {
        Json.registerType(ISubclass2.class);
    }
    String f3;
    short f4;

    @Override
    public void someFunc() {
    }

    ISubclass2() {
        set1();
    }

    void set1() {
        f3 = "1";
        f4 = 2;
    }

    void set2() {
        f3 = "3";
        f4 = 4;
    }

    public void assertEquals(ISubclass2 o2) {
        Assert.assertEquals("f3", f3, o2.f3);
        Assert.assertEquals("f4", f4, o2.f4);
    }
}

class CustomClass1 implements CustomJson {
    NestedClass1 n1 = new NestedClass1();
    int o1 = 1;

    void set2(){
        n1.c1.d1 = 1.1;
        o1 = 2;
    }

    @Override
    public void write(JsonWriter w, BuildContext context) {
        w.startObject();
        w.write("o1 custom", o1);
        w.write("subelem", n1.c1.d1);
        w.endObject();
    }

    @Override
    public void read(JsonTraverser t, ObjectIterable it, BuildContext context) {
        if (it == null) it = t.streamObject();
        for (Entry e : it) {
            if (e.key.equals("o1 custom")) o1 = (int) t.getLong();
            else if (e.key.equals("subelem")) n1.c1.d1 = t.getDouble();
            else t.unexpected(e);
        }
    }
    
    void assertEquals(CustomClass1 o2){
        Assert.assertEquals("o1", o1, o2.o1);
        n1.assertEquals(o2.n1);
    }

}

class VectorClass1 {
    Vector<SimpleClass1> vec = new Vector<>();
    int o1 = 1;
    VectorClass1() {
        set1();
    }

    void set1() {
        vec.clear();
        vec.add(new SimpleClass1());
        SimpleClass1 s2 = new SimpleClass1();
        s2.set2();
        vec.add(s2);
        SimpleClass1 s3 = new SimpleClass1();
        vec.add(s3);
    }

    void set2(){
        vec.clear();
        SimpleClass1 s1 = new SimpleClass1();
        s1.set3();
        vec.add(s1);
        SimpleClass1 s2 = new SimpleClass1();
        s2.set1();
        vec.add(s2);
        o1 = 2;
    }
    
    void assertEquals(VectorClass1 o2){
        Assert.assertEquals("o1", o1, o2.o1);
        Assert.assertEquals("vec length", vec.size(), o2.vec.size());
        for (int i = 0; i < vec.size(); ++i){
            vec.elementAt(i).assertEquals(o2.vec.elementAt(i));
        }
    }

}

enum Enum1 {
    VARIANT1,
    @JsonEntry("variant_2")
    VARIANT2,
    VARIANT3
}

class EnumClass1 {
    Enum1 f1;
    Enum1 f2;
    EnumClass1(){
        set1();
    }
    void set1(){
        f1 = Enum1.VARIANT1;
        f2 = Enum1.VARIANT2;
    }
    void set2(){
        f1 = Enum1.VARIANT3;
        f2 = Enum1.VARIANT1;
    }
    void assertEquals(EnumClass1 o2){
        Assert.assertEquals("f1", f1, o2.f1);
        Assert.assertEquals("f2", f2, o2.f2);
    }
}
