/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarc.unit.constant;

/**
 * The base type of every constant that can be stored by a constant EMAPortSymbol
 * which is used by a constant Connector
 */
public abstract class EMAConstantValue<T> {
    protected T value;

    public EMAConstantValue(T value) {
        this.value = value;
    }

    public T getValue() {
        return value;
    }

    public void setValue(T value) {
        this.value = value;
    }

    public boolean isSIUnit() {
        return false;
    }

    public boolean isBoolean() {
        return false;
    }

    @Deprecated
    public boolean isInteger() {
        return false;
    }

    @Deprecated
    public boolean isShort() {
        return false;
    }

    @Deprecated
    public boolean isLong() {
        return false;
    }

    @Deprecated
    public boolean isFloat() {
        return false;
    }

    @Deprecated
    public boolean isDouble() {
        return false;
    }

    @Deprecated
    public boolean isString() {
        return false;
    }

    @Deprecated
    public boolean isChar() {
        return false;
    }

    @Deprecated
    public boolean isByte() {
        return false;
    }

    public String getValueAsString(){
        return "null";
    }
}
