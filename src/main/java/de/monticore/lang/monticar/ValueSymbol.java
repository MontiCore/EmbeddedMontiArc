/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar;

import de.monticore.symboltable.types.references.TypeReference;

/**
 * Symbol table representation of {@link ASTValue} nodes.
 *
 * @param <T> type entry to use
 */
public class ValueSymbol<T extends TypeReference<?>> {
    /**
     * The Kind of this {@link ValueSymbol}.
     */
    protected Kind kind;
    /**
     * Type of the Enum or static field reference.
     */
    protected T type;
    /**
     * Concrete value of the argument.
     */
    protected String value;

    /**
     * Creates a new {@link ValueSymbol} that represents a value or variable.
     *
     * @param value value of the argument
     * @param kind kind of the {@link ValueSymbol}.
     */
    public ValueSymbol(final String value, final Kind kind) {
        this(null, value, kind);
    }

    /**
     * Creates a new {@link ValueSymbol} that represents a reference to an Enum value or a static
     * constant field.
     *
     * @param type type that contains the enum value/static field
     * @param value enum value name or static field name
     */
    public ValueSymbol(final T type, final String value) {
        this(type, value, Kind.ReferenceType);
    }

    /**
     * Creates a new {@link ValueSymbol} that has the given type, value, and kind.
     *
     * @param type type of this {@link ValueSymbol}
     * @param value value of this {@link ValueSymbol}
     * @param kind kind ot his {@link ValueSymbol}
     */
    public ValueSymbol(final T type, final String value, final Kind kind) {
        this.type = type;
        this.value = value;
        this.kind = kind;
    }

    /**
     * @return the kind of this {@link ValueSymbol}.
     */
    public Kind getKind() {
        return kind;
    }

    /**
     * @return the type
     */
    public T getType() {
        return type;
    }

    /**
     * @return the value
     */
    public String getValue() {
        return value;
    }

    /**
     * @return true, if this is a reference to an Enum value or a static variable, else false.
     */
    public boolean hasTypeRef() {
        return this.type != null;
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        if (hasTypeRef()) {
            sb.append(getType().getReferencedSymbol().getName());
            sb.append(".");
        }
        sb.append(getValue());
        return sb.toString();
    }

    public enum Kind {
        /**
         * {@link ValueSymbol} is a constructor call.
         */
        ConstructorCall,

        /**
         * {@link ValueSymbol} is a record.
         */
        Record,

        /**
         * {@link ValueSymbol} is a reference to a constant class or enumeration.
         */
        ReferenceType,

        /**
         * {@link ValueSymbol} is a literal value.
         */
        Value,

        /**
         * {@link ValueSymbol} is a variable name.
         */
        Variable,

        /**
         * {@link ValueSymbol} is an expression.
         */
        Expression
    }
}
