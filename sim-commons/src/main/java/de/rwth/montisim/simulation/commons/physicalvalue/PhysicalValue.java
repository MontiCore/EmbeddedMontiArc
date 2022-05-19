/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.commons.physicalvalue;

import de.rwth.montisim.commons.dynamicinterface.DataType;

/**
 * Wrapper around an Object that represents a physical value inside the
 * simulation. The goal is that multiple users can read and set the value. The
 * getter can be overloaded to hide more complex reading mechanisms for the
 * users.
 */
public abstract class PhysicalValue {
    public final transient String name;

    public PhysicalValue(String name) {
        this.name = name;
    }

    // These 2 functions can be overwritten to add some transformation on how to
    // set/get the value.
    public abstract Object get();

    public abstract void set(Object value);

    /**
     * This function returns returns the internal value. It should not be
     * overwritten.
     */
    // public abstract Object value();
    public abstract boolean hasChanged();

    public abstract DataType getType();
}