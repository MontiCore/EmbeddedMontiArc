/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.commands;

import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixAccessSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixNameExpressionSymbol;
import de.monticore.lang.monticar.generator.BluePrint;
import de.monticore.lang.monticar.generator.MathCommand;
import de.monticore.lang.monticar.generator.cpp.BluePrintCPP;
import de.monticore.lang.monticar.generator.cpp.OctaveHelper;
import de.monticore.lang.monticar.generator.cpp.converter.ComponentConverter;
import de.monticore.lang.monticar.generator.cpp.converter.ExecuteMethodGenerator;
import de.monticore.lang.monticar.generator.cpp.MathFunctionFixer;
import de.monticore.lang.monticar.generator.cpp.converter.MathConverter;
import de.monticore.lang.monticar.generator.cpp.symbols.MathStringExpression;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;

/**
 */
public class MathMaxCommand extends MathCommand {
    public MathMaxCommand(){
        setMathCommandName("max");
    }
    @Override
    public void convert(MathExpressionSymbol mathExpressionSymbol, BluePrint bluePrint) {
        String backendName = MathConverter.curBackend.getBackendName();
        if (backendName.equals("OctaveBackend")) {
            convertUsingOctaveBackend(mathExpressionSymbol, bluePrint);
        } else if (backendName.equals("ArmadilloBackend")) {
            convertUsingArmadilloBackend(mathExpressionSymbol, bluePrint);
            //Log.error("max is currently not supported in ArmadilloBackend");
        }
    }

    public void convertUsingOctaveBackend(MathExpressionSymbol mathExpressionSymbol, BluePrint bluePrint) {
        MathMatrixNameExpressionSymbol mathMatrixNameExpressionSymbol = (MathMatrixNameExpressionSymbol) mathExpressionSymbol;

        mathMatrixNameExpressionSymbol.setNameToAccess("");


        String valueListString = "";
        for (MathMatrixAccessSymbol accessSymbol : mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols())
            MathFunctionFixer.fixMathFunctions(accessSymbol, (BluePrintCPP) bluePrint);
        valueListString += ExecuteMethodGenerator.generateExecuteCode(mathExpressionSymbol, new ArrayList<String>());
        //OctaveHelper.getCallOctaveFunction(mathExpressionSymbol, "sum","Double", valueListString));
        List<MathMatrixAccessSymbol> newMatrixAccessSymbols = new ArrayList<>();
        MathStringExpression stringExpression = new MathStringExpression(OctaveHelper.getCallBuiltInFunction(mathExpressionSymbol, "Fmax", "Double", valueListString, "FirstResult", false, 1),mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols());
        newMatrixAccessSymbols.add(new MathMatrixAccessSymbol(stringExpression));

        mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().setMathMatrixAccessSymbols(newMatrixAccessSymbols);
        ((BluePrintCPP) bluePrint).addAdditionalIncludeString("octave/builtin-defun-decls");


    }
    
    public void convertUsingArmadilloBackend(MathExpressionSymbol mathExpressionSymbol, BluePrint bluePrint) {
        MathMatrixNameExpressionSymbol mathMatrixNameExpressionSymbol = (MathMatrixNameExpressionSymbol) mathExpressionSymbol;
        mathMatrixNameExpressionSymbol.setNameToAccess("");
        BluePrintCPP bluePrintCPP = (BluePrintCPP) bluePrint;
        int parametersNumber = mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols().size();

        for (MathMatrixAccessSymbol accessSymbol : mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols())
            MathFunctionFixer.fixMathFunctions(accessSymbol,bluePrintCPP);
        if (parametersNumber == 1){
            convertMaxAramadillo(mathMatrixNameExpressionSymbol, bluePrintCPP);
        }else if (parametersNumber == 2) {
            convertMaxStd(mathMatrixNameExpressionSymbol, bluePrintCPP);
        } else {
            Log.error(String.format("No implementation found for max operation: \"max(%s)\". Possible syntax is \"max( X )\", \"max(a,b)\"", mathExpressionSymbol.getTextualRepresentation()));
        }
    }

    private void convertMaxAramadillo(MathMatrixNameExpressionSymbol mathMatrixNameExpressionSymbol, BluePrintCPP bluePrint){
        String valueListString = ExecuteMethodGenerator.generateExecuteCode(mathMatrixNameExpressionSymbol, new ArrayList<>());
        MathStringExpression stringExpression = new MathStringExpression("max(max" + valueListString + ")", mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols());
        List<MathMatrixAccessSymbol> newMatrixAccessSymbols = new ArrayList<>();
        newMatrixAccessSymbols.add(new MathMatrixAccessSymbol(stringExpression));
        mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().setMathMatrixAccessSymbols(newMatrixAccessSymbols);
    }

    private void convertMaxStd(MathMatrixNameExpressionSymbol mathMatrixNameExpressionSymbol, BluePrintCPP bluePrint){
        String valueListString = ExecuteMethodGenerator.generateExecuteCode(mathMatrixNameExpressionSymbol, new ArrayList<>());
        MathStringExpression stringExpression = new MathStringExpression("std::max" + valueListString, mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols());
        List<MathMatrixAccessSymbol> newMatrixAccessSymbols = new ArrayList<>();
        newMatrixAccessSymbols.add(new MathMatrixAccessSymbol(stringExpression));
        mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().setMathMatrixAccessSymbols(newMatrixAccessSymbols);
    }
}
