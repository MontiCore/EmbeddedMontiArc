/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math._symboltable;

/**
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

