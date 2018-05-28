/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.monticore.lang.monticar;

import de.monticore.symboltable.types.references.TypeReference;

/**
 * Symbol table representation of {@link ASTValue} nodes.
 *
 * @author Arne Haber, Robert Heim
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

    /**
     * Possible {@link ValueSymbol} kinds. <br>
     * <br>
     * Copyright (c) 2017, MontiCore. All rights reserved.
     *
     * @author (last commit) $Author: ahaber $
     * $Revision: 18416 $
     */
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
