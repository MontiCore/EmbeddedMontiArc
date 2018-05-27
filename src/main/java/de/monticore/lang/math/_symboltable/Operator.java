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
package de.monticore.lang.math._symboltable;

/**
 * @author math-group
 *
 * Operator type that stores als different operators that are used in math expression (see matlab)
 */
public enum Operator {
    Plus, Minus, Times, Slash, Mod, Trans, SolEqu, PowerWise, Power, TimesWise, Div, Assign, And, Or, Equals, Nequals, Ge, Geq, Le, Leq;

    /**
     * convert the enum type based operator to a string
     *
     * @return in String {@link String} formatted operator
     */
    @Override
    public String toString() {
        switch(this) {
            case Plus: {
                return " + ";
            }
            case Trans: {
                return "\' ";
            }

            case Minus: {
                return " - ";
            }

            case Times: {
                return " * ";
            }

            case Slash: {
                return " / ";
            }

            case Mod: {
                return " % ";
            }

            case SolEqu: {
                return " // ";
            }

            case PowerWise: {
                return " .^ ";
            }

            case Power: {
                return " ^ ";
            }

            case TimesWise: {
                return " .* ";
            }

            case Div: {
                return " / ";
            }

            case Assign: {
                return " = ";
            }

            case And: {
                return " && ";
            }

            case Or: {
                return " || ";
            }

            case Equals: {
                return " == ";
            }

            case Nequals: {
                return " != ";
            }

            case Ge: {
                return " > ";
            }

            case Le: {
                return " < ";
            }

            case Geq: {
                return " >= ";
            }

            case Leq: {
                return " <= ";
            }

            default: {
                return " NoOp ";
            }
        }
    }
}

