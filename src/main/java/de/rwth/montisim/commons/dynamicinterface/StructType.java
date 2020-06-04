/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.dynamicinterface;

import java.util.HashMap;

public class StructType extends DataType {
    public String name;
    public HashMap<String, DataType> components;
    public StructType(String name){
        super(DataType.Type.STRUCT);
        this.name = name;
        components = new HashMap<>();
    }

    /** 
     * Adds a field to the struct type.
     * Note: takes ownership of the name and type.
    */
    public void addComponent(String fieldName, DataType fieldType){
        components.put(fieldName, fieldType);
    }

    @Override
    public String toString(){
        String res = name + "{ ";
        boolean first = true;
        for (HashMap.Entry<String, DataType> entry : components.entrySet()) {
            if (first){
                first = false;
            } else {
                res += ", ";
            }
            res += entry.getKey() + ": " + entry.getValue();
		}
        res += "}";
        return res;
    }

    // Implement hashCode & equals to be able to perform hashmap lookup by type & type comparison

    @Override
    public int hashCode() {
        final int prime = 31;
        int result = name.hashCode();
        for (HashMap.Entry<String, DataType> entry : components.entrySet()) {
            result = prime * result + entry.getKey().hashCode();
            result = prime * result + entry.getValue().hashCode();
		}
        return result;
    }

    @Override
    public boolean equals(Object o){
        if (o == null)
            return false;
        if (o == this)
            return true;
        if (this.getClass() != o.getClass())
            return false;
        StructType s = ((StructType)o);
        if (!this.name.equals(s.name)) return false;
        if (this.components.size() != s.components.size()) return false;
        for (HashMap.Entry<String, DataType> entry : components.entrySet()) {
            DataType t = s.components.get(entry.getKey());
            if (t == null) return false;
            if (!t.equals(entry.getValue())) return false;
		}
        return true;
    }
}