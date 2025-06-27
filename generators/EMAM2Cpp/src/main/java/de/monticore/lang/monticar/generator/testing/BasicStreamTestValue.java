/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.testing;

/**
 */
public class BasicStreamTestValue<T> implements StreamTestValue {
    T value;

    public BasicStreamTestValue(T value) {
        this.value = value;
    }

    public T getValue() {
        return value;
    }

    public void setValue(T value) {
        this.value = value;
    }

    @Override
    public String getStringRepresentation() {
        return value.toString();
    }
}
