/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.converter;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMEquationSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMInitialGuessSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMInitialValueSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMSpecificationSymbol;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.monticar.generator.cpp.loopSolver.CPPEquationSystemHelper;

import java.util.List;

public class EMAMSymbolHandler extends BaseExecuteMethodGeneratorHandler {
    @Override
    protected boolean canHandleSymbol(MathExpressionSymbol symbol) {
        if (symbol instanceof EMAMSpecificationSymbol)
            return true;
//        else if (symbol instanceof EMAMEquationSymbol)
//            return true;
//        else if (symbol instanceof EMAMInitialGuessSymbol)
//            return true;
//        else if (symbol instanceof EMAMInitialValueSymbol)
//            return true;
        else
            return false;
    }

    @Override
    protected String doGenerateExecuteCode(MathExpressionSymbol symbol, List<String> includeStrings) {
        if (symbol instanceof EMAMSpecificationSymbol)
            return doGenerateExecuteCode((EMAMSpecificationSymbol) symbol, includeStrings);
        return "";
    }

    protected String doGenerateExecuteCode(EMAMSpecificationSymbol symbol, List<String> includeStrings) {
        return "";
    }

    @Override
    public String getRole() {
        return "EMAMSymbolHandler";
    }
}
