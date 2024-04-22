package de.monticore.lang.monticar.cnnarch.gluongenerator.decomposition;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

public class LayerSubstitute {
    private ArrayList<String> allowedSubstitutes = new ArrayList<>();
    private Map<String, String> substituteAttributes = new HashMap<>();
    private String originalLayerName = null;

    public LayerSubstitute(String originalLayerName){
        this.originalLayerName = originalLayerName;
    }

    public LayerSubstitute(String originalLayerName, ArrayList<String> allowedSubstitutes){
        this.originalLayerName = originalLayerName;
        this.allowedSubstitutes = allowedSubstitutes;
    }

    public void addAttribute(String key, String value){
        if(!this.substituteAttributes.containsKey(key)){
            this.substituteAttributes.put(key, value);
        }
    }
    public void addAttribute(Map<String, String> attributes) {
        for (Map.Entry<String, String> entry : attributes.entrySet()) {
            this.addAttribute(entry.getKey(), entry.getValue());
        }
    }

    public void addSubstitute(String sub){
        this.allowedSubstitutes.add(sub);
    }

    public void addSubstituteWithAttribute(String sub, String attrName, String attrValue) {
        this.allowedSubstitutes.add(sub);
        if (!this.substituteAttributes.containsKey(attrName)) {
            this.substituteAttributes.put(attrName, attrValue);
        }
    }

    public boolean isCorrectMatch(String layer, Map<String, String> attributes) {
        if (!isAllowedSubstitute(layer)) {
            return false;
        }
        if (this.substituteAttributes.isEmpty()) {
            return true;
        }
        for (Map.Entry<String, String> entry : this.substituteAttributes.entrySet()) {
            if (!attributes.containsKey(entry.getKey()) || !attributes.get(entry.getKey()).equals(entry.getValue())) {
                return false;
            }
        }
        return true;
    }

    public boolean isAllowedSubstitute(String layer){
        for (String sub: this.allowedSubstitutes){
            if (sub.equals(layer)) return true;
        }
        return false;
    }

    public String getLayerSubstitute(String layer){
        if (isAllowedSubstitute(layer)){
            for (String sub : this.allowedSubstitutes){
                if (sub.equals(layer)) return sub;
            }
        }
        return null;
    }

    public boolean hasLayerSubstitute(String originalLayerName, String searchedSubstitute){
        for (String sub : this.allowedSubstitutes){
            if (this.originalLayerName.equals(originalLayerName) && sub.equals(searchedSubstitute)) return true;
        }
        return false;
    }

    public String getOriginalLayerName(){
        return this.originalLayerName;
    }

}
