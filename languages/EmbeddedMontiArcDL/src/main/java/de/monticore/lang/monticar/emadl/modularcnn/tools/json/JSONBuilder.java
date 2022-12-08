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

    public void addContent(String key, ArrayList<String> values,  boolean simpleValues,boolean trailing){
        this.content += JSONArray(key,values,simpleValues,trailing);
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
