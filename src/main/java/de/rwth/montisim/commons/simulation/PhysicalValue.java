/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.commons.simulation;

import de.rwth.montisim.commons.dynamicinterface.DataType;

public class PhysicalValue {
    protected Object value;
    public final String name;
    public final DataType type;
    public PhysicalValue(String name, DataType type, Object startValue){
        this.name = name;
        this.type = type;
        this.value = startValue;
    }

    // These 2 functions can be overwritten to add some transformation on how to set/get the value.
    public Object get(){
        return value;
    }
    public void set(Object value){
        this.value = value;
    }

    /** 
     * This function returns returns the internal value. 
     * It should not be overwritten.
     */
    public Object value(){
        return value;
    }
}