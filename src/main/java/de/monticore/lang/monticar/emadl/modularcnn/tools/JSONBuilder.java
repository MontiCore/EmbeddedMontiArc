package de.monticore.lang.monticar.emadl.modularcnn.tools;

import java.util.ArrayList;

public class JSONBuilder {

    public JSONBuilder(){

    }

    public static String JSONObject(String content, boolean trailing){
        return "{" + content + "}" + (trailing ? "" : ",");
    }

    public static String JSONBoolean(String key, boolean value){
       return "\"" + key + "\"" + ":" + value;
    }

    public static String JSONString(String key, String value){
        return "\"" + key + "\"" + ":" + "\"" + value + "\"";
    }

    public static String JSONEntry(String key, String value, boolean trailing){
        return JSONString(key,value) + (trailing ? "" : ",");
    }

    public static String JSONEntry(String key, boolean value, boolean trailing){
        return JSONBoolean(key,value) + (trailing ? "" : ",");
    }

    public static String JSONArray(String key, ArrayList<String> content, boolean trailing) {
        String array = "\"" + key + "\"" + ":" + "[";

        for (String element : content)
            array += content + (element.equals(content.get(content.size()-1)) ? "" : ",");

        array += "]" + (trailing ? "" : ",");

        return array;
    }


}
