/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.emadl.modularcnn.tools.json;

import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

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

    private String JSONArray(String key, ArrayList<String> values, boolean simpleValues, boolean trailing) {
        String array = "\"" + key + "\"" + ":" + "[";

        for (int i=0; i< values.size();i++){
            String element = values.get(i);
            if (simpleValues){
                element = "\"" + element + "\"";
            }
            //element = stripLeadingTrailingQuotes(element).replaceAll(" ","");
            //if (element.toCharArray()[0] != '"' || element.toCharArray()[0] != '{' || element.toCharArray()[0] != '[' ) element = "\"" + element;
            //if (element.toCharArray()[element.toCharArray().length-1] != '"' || element.toCharArray()[element.toCharArray().length-1] != '}' || element.toCharArray()[element.toCharArray().length-1] != ']') element = element + "\"";
            String newContent = "";

            if (i != values.size() -1){
                newContent = element + ",";
            } else {
                newContent = element + "";
            }

            array += newContent;
        }


        array += "]" + (trailing ? "" : ",");

        return array;
    }
    private <T> String JSONArray(String key, List<T> values, boolean trailing) {
        StringBuilder arrayBuilder = new StringBuilder();
        arrayBuilder.append("\"").append(key).append("\": [");
        for (int i = 0; i < values.size(); i++) {
            T element = values.get(i);
            String elementString;
            if (element instanceof String) {
                elementString = "\"" + element + "\"";
            } else {
                elementString = element.toString();
            }
            arrayBuilder.append(elementString);
            if (i < values.size() - 1) {
                arrayBuilder.append(",");
            }
        }
        arrayBuilder.append("]").append(trailing ? "" : ",");

        return arrayBuilder.toString();
    }

    private <K, T> String JSONMap(String key, HashMap<K, ArrayList<T>> values, boolean trailing) {
        StringBuilder mapBuilder = new StringBuilder();
        mapBuilder.append("\"").append(key).append("\": {");

        int i = 1;
        for (HashMap.Entry<K, ArrayList<T>> entry : values.entrySet()) {
            String mapKey = entry.getKey().toString();
            List<T> listValue = entry.getValue();
            mapBuilder.append(JSONArray(mapKey, listValue, i == values.size()));
            i++;
        }
        mapBuilder.append("}").append(trailing ? "" : ",");
        return mapBuilder.toString();
    }


    private String stripLeadingTrailingQuotes(String str){
        String newString = str;
        char[] charArray = str.toCharArray();

        while (charArray[0] == '"' || charArray[charArray.length-1] == '"'){
            if(charArray[0] == '"') charArray[0] = Character.MIN_VALUE;
            if(charArray[charArray.length-1] == '"') charArray[charArray.length-1] = Character.MIN_VALUE;
            newString = charArray.toString();
            charArray = newString.toCharArray();
        }

        return newString;
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
    public <T> void addContent(String key, List<T> values,boolean trailing){
        this.content += JSONArray(key,values,trailing);
    }
    public <T> void addContent(String key, HashMap<String, ArrayList<T>> values, boolean trailing) {
        this.content += JSONMap(key, values, trailing);
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
