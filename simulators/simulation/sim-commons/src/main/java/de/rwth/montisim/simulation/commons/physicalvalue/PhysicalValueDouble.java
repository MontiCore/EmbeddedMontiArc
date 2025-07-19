/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.commons.physicalvalue;

import de.rwth.montisim.commons.dynamicinterface.BasicType;
import de.rwth.montisim.commons.dynamicinterface.DataType;

public class PhysicalValueDouble extends PhysicalValue {
    public static final DataType TYPE = BasicType.DOUBLE;
    public static final double EPSILON = 0.000001;
    protected double value;
    protected double lastValue;
    protected double min;
    protected double max;

    public PhysicalValueDouble(String name, double startValue, double min, double max) {
        super(name);
        this.value = startValue;
        this.lastValue = startValue;
        this.min = min;
        this.max = max;
    }

    public PhysicalValueDouble(String name) {
        super(name);
        this.value = 0;
        this.lastValue = 0;
        this.min = Double.NEGATIVE_INFINITY;
        this.max = Double.POSITIVE_INFINITY;
    }

    public double getMin() {
        return min;
    }

    public double getMax() {
        return max;
    }

    public double getValue() {
        return value;
    }


    @Override
    public Object get() {
        return Double.valueOf(value);
    }

    @Override
    public void set(Object v) {
        lastValue = value;
        value = ((Double) v);
    }

    @Override
    public boolean hasChanged() {
        return lastValue > value + EPSILON || lastValue < value - EPSILON;
    }

    @Override
    public DataType getType() {
        return BasicType.DOUBLE;
    }
}
