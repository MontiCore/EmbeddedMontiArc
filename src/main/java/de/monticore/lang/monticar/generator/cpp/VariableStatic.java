/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.monticar.generator.Variable;

import java.util.Optional;

/**
 */
public class VariableStatic extends Variable {

    Optional<MathExpressionSymbol> assignmentSymbol;

    public VariableStatic() {
        super();
    }

    public VariableStatic(String name, String additionalInformation) {
        super(name, additionalInformation);
    }

    public VariableStatic(Variable variable) {
        super(variable);
    }

    public Optional<MathExpressionSymbol> getAssignmentSymbol() {
        return assignmentSymbol;
    }

    public void setAssignmentSymbol(MathExpressionSymbol assignmentSymbol) {
        this.assignmentSymbol = Optional.of(assignmentSymbol);
    }

}
