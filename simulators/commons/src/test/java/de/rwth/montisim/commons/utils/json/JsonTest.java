/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.utils.json;

import java.io.IOException;
import java.lang.annotation.Annotation;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;

import org.junit.Assert;
import org.junit.Test;

import de.rwth.montisim.commons.dynamicinterface.DataType;
import de.rwth.montisim.commons.utils.json.JsonTraverser.Entry;
import de.rwth.montisim.commons.utils.json.JsonTraverser.ValueType;

public class JsonTest {
    public static final String formatted = String.join("\n", "{", "  \"firstKey\": false,", "  \"sec Key\": 3,",
            "  \"t Key\": 4.2,", "  \"f Key\": \"a string \\n \\t \\r \\b \\f \\\" \\\\ \\/\",", "  \"NaN\": \"NaN\",",
            "  \"+Inf\": \"Infinity\",", "  \"-Inf\": \"-Infinity\",", "  \"skipped\": \"skipped val\",",
            "  \"empty string\": \"\",", "  \"Nested Object\": {", "    \"a\": \"b\",", "    \"also\": \"skipped\"",
            "  },", "  \"Empty Object\": {},", "  \"Array\": [", "    \"str\",", "    true,", "    {",
            "      \"c\": \"d\"", "    },", "    true", "  ],", "  \"Empty array\": []", "}");
    public static final String oneliner = "{\"firstKey\":false,\"sec Key\":3,\"t Key\":4.2,\"f Key\":\"a string \\n \\t \\r \\b \\f \\\" \\\\ \\/\",\"NaN\":\"NaN\",\"+Inf\":\"Infinity\",\"-Inf\":\"-Infinity\",\"skipped\":\"skipped val\",\"empty string\":\"\",\"Nested Object\":{\"a\":\"b\",\"also\":\"skipped\"},\"Empty Object\":{},\"Array\":[\"str\",true,{\"c\":\"d\"},true],\"Empty array\":[]}";

    @Test
    public void jsonWriterTest() {
        JsonWriter writer = new JsonWriter(true);

        writer.format = true;
        writeJsonSample(writer);
        Assert.assertEquals(formatted, writer.getString());

        writer.format = false;
        writeJsonSample(writer);
        Assert.assertEquals(oneliner, writer.getString());
    }

    public static void writeJsonSample(JsonWriter writer) {
        writer.init();

        writer.startObject();
        writer.write("firstKey", false);
        writer.write("sec Key", 3);
        writer.write("t Key", 4.2);
        writer.write("f Key", "a string \n \t \r \b \f \" \\ /");
        writer.write("NaN", Double.NaN);
        writer.write("+Inf", Double.POSITIVE_INFINITY);
        writer.write("-Inf", Double.NEGATIVE_INFINITY);

        writer.write("skipped", "skipped val");
        writer.write("empty string", "");

        writer.writeKey("Nested Object");
        writer.startObject();
        writer.write("a", "b");
        writer.write("also", "skipped");
        writer.endObject();

        writer.writeKey("Empty Object");
        writer.startObject();
        writer.endObject();

        writer.writeKey("Array");
        writer.startArray();
        writer.writeValue("str");
        writer.writeValue(true);
        writer.startObject();
        writer.write("c", "d");
        writer.endObject();
        writer.writeValue(true);
        writer.endArray();

        writer.writeKey("Empty array");
        writer.startArray();
        writer.endArray();

        writer.endObject();

        // System.out.println(writer.getString());
    }

    @Test
    public void jsonTraverserTest() throws IOException {
        traverse(formatted);
        traverse(oneliner);

        // System.out.println("DOUBLE annotations");
        // try {
        //     for (Annotation ann : DataType.Type.class.getDeclaredField("DOUBLE").getAnnotations()) {
        //         System.out.println(ann.toString());
        //     }
        // } catch (NoSuchFieldException | SecurityException e) {
        //     e.printStackTrace();
        // }
        

        // System.out.println("BOOLEAN annotations");
        // for (Annotation ann : DataType.BOOLEAN.getClass().getAnnotations()){
        //     System.out.println(ann.toString());
        // }
        // System.out.println("Vec2 annotations");
        // for (Annotation ann : Vec2.class.getAnnotations()){
        //     System.out.println(ann.toString());
        // }
        
        // System.out.println("DynamicObject fields");
        // for(Field f : DynamicObject.class.getDeclaredFields()){
        //     if(!Modifier.isStatic(f.getModifiers()))
        //         System.out.println(f.toString());
        // }
    }

    public void traverse(String data) throws IOException {
        JsonTraverser j = new JsonTraverser();
        j.init(data);

        boolean v1 = true;
        long v2 = 0;
        double v3 = 0.0;
        String v4 = "";
        double v5 = 0, v6 = 0, v7 = 0;
        String v8 = "", v9 = "";
        String v11 = "";
        String a = "content";

        for (Entry e : j.streamObject()){
            if (e.key.equals("firstKey")){
                v1 = j.getBoolean();
            } else if (e.key.equals("sec Key")){
                v2 = j.getLong();
            } else if (e.key.equals("t Key")){
                v3 = j.getDouble();
            } else if (e.key.equals("f Key")){
                v4 = j.getString().getJsonString();
            } else if (e.key.equals("NaN")){
                v5 = j.getDouble();
            } else if (e.key.equals("+Inf")){
                v6 = j.getDouble();
            } else if (e.key.equals("-Inf")){
                v7 = j.getDouble();
            } else if (e.key.equals("empty string")){
                a = j.getString().getJsonString();
            } else if (e.key.equals("skipped")){
                //Just skip
            } else if (e.key.equals("Nested Object")){
                for (Entry e2 : j.streamObject()){
                    if (e2.key.equals("a")){
                        v8 = j.getString().getJsonString();
                    } else if (e2.key.equals("also")){
                        //Skip
                    } else throw new RuntimeException("Unexpected entry: "+e2.key.getRawString());
                }
            } else if (e.key.equals("Empty Object")){
                for (Entry e2 : j.streamObject()){
                    throw new RuntimeException("Unexpected entry: "+e2.key.getRawString());
                }
            } else if (e.key.equals("Array")){
                for (ValueType t : j.streamArray()){
                    if (t == ValueType.STRING){
                        v9 = j.getString().getJsonString();
                    } else if (t == ValueType.BOOLEAN){
                        // Test skipping elems in array
                    } else if (t == ValueType.OBJECT){
                        for (Entry e2 : j.streamObject()){
                            if (e2.key.equals("c")){
                                v11 = j.getString().getJsonString();
                            }
                        }
                    } else throw new RuntimeException("Unexpected entry type: "+t);
                }
            } else if (e.key.equals("Empty array")){
                for (ValueType t : j.streamArray()){
                    throw new RuntimeException("Unexpected entry in empty array");
                }
            } else {
                throw new RuntimeException("Unexpected entry: "+e.key.getRawString());
            }
        }

        Assert.assertFalse("Error getting 'firstKey'", v1);
        Assert.assertEquals("Error getting 'sec Key'", 3, v2);
        Assert.assertEquals("Error getting 't Key'", 4.2, v3, 0.00001);
        Assert.assertEquals("Error getting 'f Key'", "a string \n \t \r \b \f \" \\ /", v4);
        Assert.assertTrue("Error getting 'NaN'", Double.isNaN(v5));
        Assert.assertTrue("Error getting '+Inf'", Double.isInfinite(v6) && v6 >0);
        Assert.assertTrue("Error getting '-Inf'", Double.isInfinite(v7) && v7 <0);
        Assert.assertEquals("Error getting 'Nested Object'", "b", v8);
        Assert.assertEquals("Error getting 'Array->str'", "str", v9);
        Assert.assertEquals("Error getting 'Array->Nested Object'", "d", v11);
        Assert.assertEquals("Error getting 'empty string'", "", a);


    }
}
