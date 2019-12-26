/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.commands;

import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixAccessSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixNameExpressionSymbol;
import de.monticore.lang.monticar.generator.BluePrint;
import de.monticore.lang.monticar.generator.MathCommand;
import de.monticore.lang.monticar.generator.cpp.BluePrintCPP;
import de.monticore.lang.monticar.generator.cpp.MathFunctionFixer;
import de.monticore.lang.monticar.generator.cpp.OctaveHelper;
import de.monticore.lang.monticar.generator.cpp.converter.ExecuteMethodGenerator;
import de.monticore.lang.monticar.generator.cpp.converter.MathConverter;
import de.monticore.lang.monticar.generator.cpp.symbols.MathStringExpression;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;

/**
 * @author Ahmed Diab.
 */
public abstract class ArgumentReturnMathCommand extends MathCommand{
    public ArgumentReturnMathCommand() {

    }

    public ArgumentReturnMathCommand(String argumentReturnMathCommandName) {
        super(argumentReturnMathCommandName);
    }

    @Override
    public boolean isArgumentReturnMathCommand() {
        return true;
    }
}
