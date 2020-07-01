package de.rwth.montisim.commons.utils.json;

import java.io.IOException;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;

import org.junit.Assert;
import org.junit.Test;


public class AnnotatedJsonTest {

    @Test
    public void simpleTest1() throws IllegalArgumentException, IllegalAccessException {
        SimpleClass1 o = new SimpleClass1();

        Json j = Json.formattedWriter();
        j.toJson(o);

        String res = j.w.getString();
        // System.out.println(res);
        Assert.assertEquals("{\n" + "  \"s1\": \"a string\",\n" + "  \"double1\": 5.6\n" + "}", res);
    }

    @Test
    public void simpleTest2() throws IllegalArgumentException, IllegalAccessException {
        SimpleClass2 o = new SimpleClass2();

        Json j = Json.formattedWriter();
        j.toJson(o);

        String res = j.w.getString();
        // System.out.println(res);
        Assert.assertEquals("{\n" + "  \"s1\": \"abcde\",\n" + "  \"i1\": 4,\n" + "  \"d1\": 5.6,\n"
                + "  \"aaa\": 4.2,\n" + "  \"c1\": 65,\n" + "  \"sh1\": 10,\n" + "  \"l1\": -5,\n" + "  \"b1\": 8,\n"
                + "  \"z1\": true\n" + "}", res);

    }

    @Test
    public void emptyTest1() throws IllegalArgumentException, IllegalAccessException {
        EmptyClass1 o = new EmptyClass1();

        Json j = Json.formattedWriter();
        j.toJson(o);

        String res = j.w.getString();
        // System.out.println(res);
        Assert.assertEquals("{}", res);

    }

    @Test
    public void simpleTest3() throws IllegalArgumentException, IllegalAccessException {
        SimpleClass3 o = new SimpleClass3();

        Json j = Json.formattedWriter();
        j.toJson(o);

        String res = j.w.getString();
        // System.out.println(res);
        Assert.assertEquals("[\n" + "  \"text\",\n" + "  1,\n" + "  100.0\n" + "]", res);

    }

    @Test
    public void nestedTest1() throws IllegalArgumentException, IllegalAccessException {
        NestedClass1 o = new NestedClass1();

        Json j = Json.formattedWriter();
        j.toJson(o);

        String res = j.w.getString();
        // System.out.println(res);
        Assert.assertEquals("{\n" + "  \"s1\": \"abcde\",\n" + "  \"c1\": {\n" + "    \"s1\": \"a string\",\n"
                + "    \"double1\": 5.6\n" + "  },\n" + "  \"nested_entry\": {\n" + "    \"s1\": \"abcde\",\n"
                + "    \"i1\": 4,\n" + "    \"d1\": 5.6,\n" + "    \"aaa\": 4.2,\n" + "    \"c1\": 65,\n"
                + "    \"sh1\": 10,\n" + "    \"l1\": -5,\n" + "    \"b1\": 8,\n" + "    \"z1\": true\n" + "  },\n"
                + "  \"e1\": {},\n" + "  \"arraySubClass\": [\n" + "    \"text\",\n" + "    1,\n" + "    100.0\n"
                + "  ],\n" + "  \"z1\": true\n" + "}", res);

    }

    @Test
    public void typedTest1() throws IllegalArgumentException, IllegalAccessException {
        TypedClass1 o = new TypedClass1();

        Json j = Json.formattedWriter();
        j.toJson(o);

        String res = j.w.getString();
        // System.out.println(res);
        Assert.assertEquals("{\n" + "  \"type\": \"TypedClass1\",\n" + "  \"a\": 1,\n" + "  \"b\": 2\n" + "}", res);
    }

    @Test
    public void typedTest2() throws IllegalArgumentException, IllegalAccessException {
        TypedClass2 o = new TypedClass2();

        Json j = Json.formattedWriter();
        j.toJson(o);

        String res = j.w.getString();
        // System.out.println(res);
        Assert.assertEquals("{\n" + "  \"type\": \"type2\",\n" + "  \"a\": 1,\n" + "  \"b\": 2\n" + "}", res);
    }

    @Test
    public void subclassTest1() throws IllegalArgumentException, IllegalAccessException {
        Subclass1 o = new Subclass1();

        Json j = Json.formattedWriter();
        j.toJson(o);

        String res = j.w.getString();
        // System.out.println(res);
        Assert.assertEquals(
                "{\n" + "  \"s1\": \"a string\",\n" + "  \"double1\": 5.6,\n" + "  \"a\": 1,\n" + "  \"b\": 2\n" + "}",
                res);
    }

    @Test
    public void subclassTest2() throws IllegalArgumentException, IllegalAccessException {
        Subclass2 o = new Subclass2();

        Json j = Json.formattedWriter();
        j.toJson(o);

        String res = j.w.getString();
        //System.out.println(res);
        Assert.assertEquals("{\n" + "  \"type\": \"sub2\",\n" + "  \"s1\": \"a string\",\n" + "  \"double1\": 5.6,\n"
                + "  \"a\": 1,\n" + "  \"b\": 2\n" + "}", res);
    }

    @Test
    public void instanciateTest1() throws InstantiationException, IllegalAccessException, IllegalArgumentException,
            InvocationTargetException, NoSuchMethodException, SecurityException, IOException {
        SimpleClass1 o = new SimpleClass1();
        o.set2();

        // System.out.println("Constructors");
        // for (Constructor c : SimpleClass1.class.getDeclaredConstructors()){
        //     System.out.println(c);
        // }        

        Json j = Json.formattedWriter();
        j.toJson(o);

        String res = j.w.getString();
        //System.out.println(res);
        Json j2 = Json.traverser(res);
        SimpleClass1 o2 = j2.instanciateFromJson(SimpleClass1.class);
        o.assertEquals(o2);
    }

    @Test
    public void instanciateTest2() throws InstantiationException, IllegalAccessException, IllegalArgumentException,
            InvocationTargetException, NoSuchMethodException, SecurityException, IOException {
        SimpleClass2 o = new SimpleClass2();
        o.set2();

        // System.out.println("Constructors");
        // for (Constructor c : SimpleClass1.class.getDeclaredConstructors()){
        //     System.out.println(c);
        // }        

        Json j = Json.formattedWriter();
        j.toJson(o);

        String res = j.w.getString();
        //System.out.println(res);
        Json j2 = Json.traverser(res);
        SimpleClass2 o2 = j2.instanciateFromJson(SimpleClass2.class);
        o.assertEquals(o2);
    }

    @Test
    public void instanciateTest3() throws InstantiationException, IllegalAccessException, IllegalArgumentException,
            InvocationTargetException, NoSuchMethodException, SecurityException, IOException {
        SimpleClass3 o = new SimpleClass3();
        o.set2();

        // System.out.println("Fields");
        // for (Field f : SimpleClass3.class.getDeclaredFields()){
        //     System.out.println(f);
        // }        

        Json j = Json.formattedWriter();
        j.toJson(o);

        String res = j.w.getString();
        //System.out.println(res);
        Json j2 = Json.traverser(res);
        SimpleClass3 o2 = j2.instanciateFromJson(SimpleClass3.class);
        o.assertEquals(o2);
    }

    @Test
    public void instanciateNestedTest() throws InstantiationException, IllegalAccessException, IllegalArgumentException,
            InvocationTargetException, NoSuchMethodException, SecurityException, IOException {
        NestedClass1 o = new NestedClass1();
        o.set2();

        // System.out.println("Constructors");
        // for (Constructor c : SimpleClass1.class.getDeclaredConstructors()){
        //     System.out.println(c);
        // }        

        Json j = Json.formattedWriter();
        j.toJson(o);

        String res = j.w.getString();
        //System.out.println(res);
        Json j2 = Json.traverser(res);
        NestedClass1 o2 = j2.instanciateFromJson(NestedClass1.class);
        o.assertEquals(o2);
    }

    // TODO Test "typed"

    // TODO test subclasses

    // TODO test typed subclasses

    // TODO test instanciate from super type into subtype variants
}

@Jsonable
class SimpleClass1 {
    @JsonEntry
    String s1;
    int i1; // Is NOT serialized
    @JsonEntry("double1")
    double d1;

    SimpleClass1(){
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

    public void assertEquals(SimpleClass1 o2) {
        Assert.assertEquals("s1", s1, o2.s1);
        Assert.assertEquals("d1", d1, o2.d1, 0.00001);
    }
}

@Jsonable(fields=FieldSelect.ALL)
class SimpleClass2 {
    String s1;
    int i1;
    double d1;
    @JsonEntry("aaa")
    float f1;
    char c1;
    //@IgnoreField
    transient String s2 = "chocolate";
    short sh1;
    long l1;
    byte b1;
    boolean z1;

    SimpleClass2(){
        set1();
    }

    void set1(){
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

    void set2(){
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

@Jsonable
class EmptyClass1 {
    String s1;
    int i1;
    double d1;
}

@Jsonable(fields=FieldSelect.ALL,type=StructureType.ARRAY)
class SimpleClass3 {
    String s1;
    private int i1;
    double d1;

    SimpleClass3(){
        set1();
    }

    void set1(){
        s1 = "text";
        i1 = 1;
        d1 = 100;
    }
    void set2(){
        s1 = "other text";
        i1 = 78;
        d1 = -100.1;
    }

    public void assertEquals(SimpleClass3 o2) {
        Assert.assertEquals("s1", s1, o2.s1);
        Assert.assertEquals("d1", d1, o2.d1, 0.00001);
    }
}

@Jsonable(fields=FieldSelect.ALL)
class NestedClass1 {
    String s1;
    SimpleClass1 c1;
    @JsonEntry("nested_entry")
    SimpleClass2 c2;
    //@IgnoreField
    transient SimpleClass2 c3;
    EmptyClass1 e1;
    SimpleClass3 arraySubClass;
    boolean z1;

    NestedClass1(){
        set1();
    }

    void set1(){
        s1 = "abcde";
        c1 = new SimpleClass1();
        c2 = new SimpleClass2();
        e1 = new EmptyClass1();
        arraySubClass = new SimpleClass3();
        z1 = true;
    }

    void set2(){
        s1 = "edcba";
        c1 = new SimpleClass1();
        c1.set2();
        c2 = new SimpleClass2();
        c2.set2();
        e1 = new EmptyClass1();
        arraySubClass = new SimpleClass3();
        arraySubClass.set2();
        z1 = false;
    }

    public void assertEquals(NestedClass1 o2) {
        Assert.assertEquals("s1", s1, o2.s1);
        c1.assertEquals(o2.c1);
        c2.assertEquals(o2.c2);
        arraySubClass.assertEquals(o2.arraySubClass);
        Assert.assertEquals("z1", z1, o2.z1);
    }
}

@Jsonable(fields=FieldSelect.ALL)
@Typed
class TypedClass1 {
    int a;
    int b;
    TypedClass1(){
        set1();
    }
    void set1(){
        a = 1;
        b = 2;
    }
}

@Jsonable(fields=FieldSelect.ALL)
@Typed("type2")
class TypedClass2 {
    int a;
    int b;
    TypedClass2(){
        set1();
    }
    void set1(){
        a = 1;
        b = 2;
    }
}

@Jsonable(fields=FieldSelect.ALL)
class Subclass1 extends SimpleClass1 {
    int a;
    int b;
    Subclass1(){
        sset1();
    }
    void sset1(){
        a = 1;
        b = 2;
    }
}

@Jsonable(fields=FieldSelect.ALL)
@Typed("sub2")
class Subclass2 extends SimpleClass1 {
    int a;
    int b;
    Subclass2(){
        sset1();
    }
    void sset1(){
        a = 1;
        b = 2;
    }
}