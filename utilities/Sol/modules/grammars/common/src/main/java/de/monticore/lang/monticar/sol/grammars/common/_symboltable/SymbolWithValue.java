/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.common._symboltable;

import java.util.Optional;

/**
 * An interface to be implemented by symbols holding a primitive value.
 */
public interface SymbolWithValue extends SymbolWithType {
    /**
     * Replaces the value with the given value.
     * @param value The value with which the existing value should be replaced.
     */
    void setValue(Object value);

    /**
     * The value of the symbol wrapped in an Optional.
     * @return An Optional holding the value of the symbol.
     */
    Optional<Object> getValue();

    /**
     * Returns the type of the value.
     * @return The value of the type.
     */
    default Optional<String> getType() {
        if (this.isBoolean()) return Optional.of("boolean");
        else if (this.isString()) return Optional.of("string");
        else if (this.isNumber()) return Optional.of("number");
        else return Optional.empty();
    }

    /**
     * Checks whether the value is a Boolean.
     * @return True if the value is a Boolean, false otherwise.
     */
    default boolean isBoolean() {
        return this.getValue()
                .map(value -> value instanceof Boolean)
                .orElse(false);
    }

    /**
     * Checks whether the value is a String.
     * @return True if the value is a String, false otherwise.
     */
    default boolean isString() {
        return this.getValue()
                .map(value -> value instanceof String)
                .orElse(false);
    }

    /**
     * Checks whether the value is a Number.
     * @return True if the value is a Number, false otherwise.
     */
    default boolean isNumber() {
        return this.getValue()
                .map(value -> value instanceof Number)
                .orElse(false);
    }

    /**
     * The value as Boolean wrapped in Optional if the value is indeed a Boolean.
     * @return The value as Boolean wrapped in Optional. The Optional is empty if the value is not a Boolean.
     */
    default Optional<Boolean> getValueAsBoolean() {
        return this.getValue()
                .filter(value -> value instanceof Boolean)
                .map(value -> (Boolean)value);
    }

    /**
     * The value as String wrapped in Optional if the value is indeed a String.
     * @return The value as String wrapped in Optional. The Optional is empty if the value is not a String.
     */
    default Optional<String> getValueAsString() {
        return this.getValue()
                .filter(value -> value instanceof String)
                .map(value -> (String)value);
    }

    /**
     * The value as Number wrapped in Optional if the value is indeed a Number.
     * @return The value as Number wrapped in Optional. The Optional is empty if the value is not a Number.
     */
    default Optional<Number> getValueAsNumber() {
        return this.getValue()
                .filter(value -> value instanceof Number)
                .map(value -> (Number)value);
    }
}
