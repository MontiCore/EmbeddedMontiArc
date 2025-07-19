/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.solver.solutionValidation.arithmetic;

import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.monticar.semantics.solver.solutionValidation.ArithmeticValidationViewModel;

import java.util.List;

public interface ArithmeticValidation {
    public List<ArithmeticValidationViewModel> getValids(MathExpressionSymbol expression);

    public List<ArithmeticValidationViewModel> getInvalids(MathExpressionSymbol expression);
}
