/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.utils.json;

import java.io.IOException;

import de.rwth.montisim.commons.utils.json.JsonTraverser.*;

public class JsonWriter {

    public JsonWriter(boolean format){
        init();
        this.format = format;
    }

    public static final int TAB = 2;
    public StringBuilder res;
    public boolean format = true;
    int offset = 0;
    boolean hasElem = false;
    boolean hasKey = true;

    public void init(){
        res = new StringBuilder();
        offset = 0;
        hasElem = false;
        hasKey = true;
    }

    public String getString(){
        return res.toString();
    }

    public void startObject() {
        separate();
        if (format) offset += TAB;
        res.append('{');
        hasElem = false;
    }

    private void addOffset(){
        for (int i = 0; i<offset; ++i){
            res.append(' ');
        }
    }

    public void endObject() {
        if (format) {
            offset -= TAB;
            if (hasElem){
                res.append('\n');
                addOffset();
            }
        }
        res.append('}');
        hasElem = true;
    }

    public void startArray() {
        separate();
        if (format) offset += TAB;
        res.append('[');
        hasElem = false;
    }

    public void endArray() {
        if (format) {
            offset -= TAB;
            if (hasElem){
                res.append('\n');
                addOffset();
            }
        }
        res.append(']');
        hasElem = true;
    }

    private void separate() {
        if (hasElem && !hasKey) res.append(',');
        else hasElem = true;
        
        if (!hasKey) {
            if (format) {
                res.append('\n');
                addOffset();
            }
        }
        else hasKey = false;
    }

    public void writeKey(String key) {
        writeValue(key);
        hasKey = true;
        if (format) res.append(": ");
        else res.append(':');
    }

    public void writeValue(String str) {
        separate();
        res.append('"');
        for (int i = 0; i < str.length(); ++i){
            char c = str.charAt(i);
            if (c == '\n'){
                res.append("\\n");
            } else if (c == '\t'){
                res.append("\\t");
            } else if (c == '\r'){
                res.append("\\r");
            } else if (c == '\f'){
                res.append("\\f");
            } else if (c == '\b'){
                res.append("\\b");
            } else if (c == '"'){
                res.append("\\\"");
            } else if (c == '/'){
                res.append("\\/");
            } else if (c == '\\'){
                res.append("\\\\");
            } else 
            res.append(c);
        }
        res.append('"');
    }

    public void writeValue(int i){
        separate();
        res.append(i);
    }
    public void writeValue(long l){
        separate();
        res.append(l);
    }
    public void writeValue(double d){
        separate();
        if (Double.isNaN(d)) res.append("\"NaN\"");
        else if (Double.isInfinite(d)) {
            res.append('"');
            res.append(d);
            res.append('"');
        }
        else res.append(d);
    }
    public void writeValue(float f){
        separate();
        if (Double.isNaN(f)) res.append("\"NaN\"");
        else if (Double.isInfinite(f)) {
            res.append('"');
            res.append(f);
            res.append('"');
        }
        else res.append(f);
    }
    public void writeValue(boolean b){
        separate();
        if(b) {
            res.append("true");
        } else {
            res.append("false");
        }
    }

    public void write(String key, String str){
        writeKey(key);
        writeValue(str);
    }

    public void write(String key, int i){
        writeKey(key);
        writeValue(i);
    }

    public void write(String key, long l){
        writeKey(key);
        writeValue(l);
    }
    public void write(String key, float f){
        writeKey(key);
        writeValue(f);
    }
    public void write(String key, double d){
        writeKey(key);
        writeValue(d);
    }
    public void write(String key, boolean b){
        writeKey(key);
        writeValue(b);
    }


    public static void rewrite(JsonWriter w, JsonTraverser j){
        switch(j.getType()){
            case ARRAY:
                w.startArray();
                for (ValueType t : j.streamArray()){
                    rewrite(w, j);
                }
                w.endArray();
                break;
            case BOOLEAN:
                w.writeValue(j.getBoolean());
                break;
            case NUMBER:
                w.writeValue(j.getDouble());
                break;
            case OBJECT:
                w.startObject();
                for (Entry e : j.streamObject()){
                    w.writeKey(e.key.getJsonString());
                    rewrite(w, j);
                }
                w.endObject();
                break;
            case STRING:
                w.writeValue(j.getString().getJsonString());
                break;
            case UNKNOWN:
                break;
            default:
                break;
        }
    }





    
    public static void main(String args[]) throws IOException {
        JsonWriter writer = new JsonWriter(true);
        jsonTest(writer);

        String s1 = writer.getString();

        writer.format = false;
        jsonTest(writer);

        String s2 = writer.getString();

        JsonTraverser j = new JsonTraverser();
        j.init(s1);
        writer.init();
        rewrite(writer, j);

        String s3 = writer.getString();

        System.out.println("Re-written");
        System.out.println(s3);
    }

    public static void jsonTest(JsonWriter writer){
        writer.init();

        writer.startObject();
        writer.writeKey("firstKey");
        writer.writeValue(false);
        writer.writeKey("sec Key");
        writer.writeValue(3);
        writer.writeKey("t Key");
        writer.writeValue(4.2);
        writer.writeKey("f Key");
        writer.writeValue("a string \n \t \r \b \f \" \\ /");
        writer.writeKey("NaN");
        writer.writeValue(Double.NaN);
        writer.writeKey("+Inf");
        writer.writeValue(Double.POSITIVE_INFINITY);
        writer.writeKey("-Inf");
        writer.writeValue(Double.NEGATIVE_INFINITY);

        writer.writeKey("Nested Object");
        writer.startObject();
        writer.writeKey("a");
        writer.writeValue("b");
        writer.endObject();
        
        writer.writeKey("Empty Object");
        writer.startObject();
        writer.endObject();

        writer.writeKey("Array");
        writer.startArray();
        writer.writeValue("str");
        writer.writeValue(true);
        writer.writeKey("Nested Object");
        writer.startObject();
        writer.writeKey("a");
        writer.writeValue("b");
        writer.endObject();
        writer.writeValue(true);
        writer.endArray();

        writer.writeKey("Empty array");
        writer.startArray();
        writer.endArray();

        writer.endObject();

        System.out.println(writer.getString());
    }
}
