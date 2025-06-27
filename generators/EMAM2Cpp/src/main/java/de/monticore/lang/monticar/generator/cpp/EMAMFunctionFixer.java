/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMSpecificationSymbol;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.monticar.generator.*;
import de.monticore.lang.monticar.generator.cpp.loopSolver.CPPEquationSystemHelper;
import de.monticore.lang.monticar.generator.cpp.loopSolver.DummyRHSComponent;
import de.monticore.lang.monticar.generator.testing.StreamTestGenerator;
import de.monticore.lang.monticar.semantics.loops.symbols.EMAEquationSystemHelper;
import de.monticore.lang.monticar.semantics.loops.symbols.semiexplicit.SemiExplicitForm;

public class EMAMFunctionFixer extends BaseMathFunctionFixerHandler {
    @Override
    protected boolean canFixMathSymbol(MathExpressionSymbol symbol) {
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
    protected void doFixMathFunction(MathExpressionSymbol symbol, EMAMBluePrintCPP bluePrintCPP) {
        if (symbol instanceof EMAMSpecificationSymbol)
            doFixMathFunction((EMAMSpecificationSymbol) symbol, bluePrintCPP);
    }

    protected void doFixMathFunction(EMAMSpecificationSymbol symbol, EMAMBluePrintCPP bluePrintCPP) {
        EMAComponentInstanceSymbol originalSymbol = bluePrintCPP.getOriginalSymbol();
        SemiExplicitForm semiExplicitForm = EMAEquationSystemHelper.buildSemiExplicitForm(originalSymbol, symbol);
        GeneratorCPP generator = (GeneratorCPP) bluePrintCPP.getGenerator();
        CPPEquationSystemHelper.handleEquationSystem(semiExplicitForm, bluePrintCPP,
                generator, bluePrintCPP.getAdditionalUserIncludeStrings());

        DummyRHSComponent rhsComponent = new DummyRHSComponent(originalSymbol);
        StreamTestGenerator streamTestGenerator = new StreamTestGenerator();//only used when creating streamTestsForAComponent
        LanguageUnitCPP languageUnitCPP = new LanguageUnitCPP();
        languageUnitCPP.setGeneratorCPP(generator);
        languageUnitCPP.addSymbolToConvert(rhsComponent);
        languageUnitCPP.generateBluePrints();
        EMAMBluePrintCPP currentBluePrint = null;
        for (EMAMBluePrint bluePrint : languageUnitCPP.getBluePrints())
            if (bluePrint.getOriginalSymbol().equals(rhsComponent))
                bluePrintCPP = (EMAMBluePrintCPP) bluePrint;

        CPPEquationSystemHelper.handleRHS(semiExplicitForm, bluePrintCPP, generator, bluePrintCPP.getAdditionalUserIncludeStrings());

        String result = languageUnitCPP.getGeneratedHeader(null, bluePrintCPP);
        generator.addFileContent(new FileContent(result, rhsComponent));
    }

    @Override
    public String getRole() {
        return "EMAMFunctionFixer";
    }
}
