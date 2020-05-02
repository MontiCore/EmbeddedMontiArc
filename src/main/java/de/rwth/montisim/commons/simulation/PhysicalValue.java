/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.commons.simulation;

public class PhysicalValue {
    protected double value;
    public final String name;
    public PhysicalValue(String name, double startValue){
        this.value = startValue;
        this.name = name;
    }

    // These 2 functions can be overwritten to add some transformation on how to set/get the value.
    public double get(){
        return value;
    }
    public void set(double value){
        this.value = value;
    }

    /** 
     * This function returns returns the internal value. 
     * It should not be overwritten.
     */
    public double value(){
        return value;
    }
}