/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.emadl.modularcnn.tools.json;

import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;

public class JSONBuilder {

    private String content = "";

    public JSONBuilder(){

    }

    private String JSONObject(boolean trailing){
        return "{" + this.content + "}" + (trailing ? "" : ",");
    }

    private String JSONBoolean(String key, boolean value){
       return "\"" + key + "\"" + ":" + value;
    }

    private String JSONString(String key, String value){
        return "\"" + key + "\"" + ":" + "\"" + value + "\"";
    }

    private String JSONEntry(String key, String value, boolean trailing){
        return JSONString(key,value) + (trailing ? "" : ",");
    }

    private String JSONEntry(String key, boolean value, boolean trailing){
        return JSONBoolean(key,value) + (trailing ? "" : ",");
    }

    private String JSONArray(String key, ArrayList<String> values, boolean trailing) {
        String array = "\"" + key + "\"" + ":" + "[";

        for (String element : values)
            array += element + (element.equals(values.get(values.size()-1)) ? "" : ",");

        array += "]" + (trailing ? "" : ",");

        return array;
    }

    private void reset(){
        this.content = "";
    }

    public void addContent(String key, String value, boolean trailing){
        this.content += JSONEntry(key,value,trailing);
    }

    public void addContent(String key, boolean value, boolean trailing){
        this.content += JSONEntry(key,value,trailing);
    }

    public void addContent(String key, ArrayList<String> values, boolean trailing){
        this.content += JSONArray(key,values,trailing);
    }

    public String getJSONObject(boolean trailing){
        return JSONObject(trailing).replaceAll(" ", "").replaceAll("\n","");
    }

    public String getJSONObjectAndReset(boolean trailing){
        String json = getJSONObject(trailing);
        reset();
        return json;
    }







}
