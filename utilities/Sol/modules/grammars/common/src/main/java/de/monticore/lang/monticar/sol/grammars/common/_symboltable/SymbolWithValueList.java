/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.common._symboltable;

import java.util.List;
import java.util.stream.Collectors;

/**
 * An interface to be implemented by symbols holding multiple primitive values.
 */
public interface SymbolWithValueList extends SymbolWithType {
    /**
     * Adds a primitive value to the list of values.
     * @param value The value to be added.
     */
    void addValue(Object value);

    /**
     * Removes a primitive value from the list of values.
     * @param value The value to be removed.
     */
    void removeValue(Object value);

    /**
     * Sets the list of values to the one given as parameter.
     * @param values The values to which the list of values should be set.
     */
    void setValues(List<Object> values);

    /**
     * Adds the values given as parameters to the list of values.
     * @param values The values to be added.
     */
    void addValues(List<Object> values);

    /**
     * Returns the list of values.
     * @return The list of values.
     */
    List<Object> getValues();

    /**
     * Checks whether the list of values is a list of Booleans.
     * @return True if the list of values is a list of Booleans.
     */
    default boolean isBooleanList() {
        return this.getType().isPresent() && this.getType().get().equals("boolean[]") && this.getValues().stream()
                .map(value -> value instanceof Boolean)
                .reduce(true, (v1, v2) -> v1 && v2);
    }

    /**
     * Checks whether the list of values is a list of Strings.
     * @return True if the list of values is a list of Strings.
     */
    default boolean isStringList() {
        return this.getType().isPresent() && this.getType().get().equals("string[]") && this.getValues().stream()
                .map(value -> value instanceof String)
                .reduce(true, (v1, v2) -> v1 && v2);
    }

    /**
     * Checks whether the list of values is a list of Numbers.
     * @return True if the list of values is a list of Numbers.
     */
    default boolean isNumberList() {
        return this.getType().isPresent() && this.getType().get().equals("number[]") && this.getValues().stream()
                .map(value -> value instanceof Number)
                .reduce(true, (v1, v2) -> v1 && v2);
    }

    /**
     * Return the list of values as list of Booleans if it is indeed a list of Booleans.
     * @return The list of values as list of Booleans or an empty list if it is not a list of Booleans.
     */
    default List<Boolean> getValuesAsBooleans() { // TODO: Throw exception instead of returning an empty list.
        return this.getValues().stream()
                .filter(value -> value instanceof Boolean)
                .map(value -> (Boolean)value)
                .collect(Collectors.toList());
    }

    /**
     * Return the list of values as list of Strings if it is indeed a list of Strings.
     * @return The list of values as list of Strings or an empty list if it is not a list of Strings.
     */
    default List<String> getValuesAsStrings() {
        return this.getValues().stream()
                .filter(value -> value instanceof String)
                .map(value -> (String)value)
                .collect(Collectors.toList());
    }

    /**
     * Return the list of values as list of Numbers if it is indeed a list of Numbers.
     * @return The list of values as list of Numbers or an empty list if it is not a list of Numbers.
     */
    default List<Number> getValuesAsNumbers() {
        return this.getValues().stream()
                .filter(value -> value instanceof Number)
                .map(value -> (Number)value)
                .collect(Collectors.toList());
    }
}
