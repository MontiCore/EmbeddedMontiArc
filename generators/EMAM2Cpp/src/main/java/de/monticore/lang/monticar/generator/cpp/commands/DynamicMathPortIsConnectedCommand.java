/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.commands;

import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixAccessSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixNameExpressionSymbol;
import de.monticore.lang.monticar.generator.EMAMBluePrint;
import de.monticore.lang.monticar.generator.MathCommand;
import de.monticore.lang.monticar.generator.cpp.EMAMBluePrintCPP;
import de.monticore.lang.monticar.generator.cpp.MathFunctionFixer;
import de.monticore.lang.monticar.generator.cpp.converter.ExecuteMethodGenerator;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.MathStringExpression;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;

public class DynamicMathPortIsConnectedCommand extends MathCommand {

    public DynamicMathPortIsConnectedCommand() {
        setMathCommandName("is_connected");
    }

    @Override
    protected void convert(MathExpressionSymbol mathExpressionSymbol, EMAMBluePrint bluePrint) {
        MathMatrixNameExpressionSymbol mathMatrixNameExpressionSymbol = (MathMatrixNameExpressionSymbol) mathExpressionSymbol;

        List<MathMatrixAccessSymbol> symbols = mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols();
        if(symbols.size() != 2){
            Log.error("Is connected wrong number of arguments! Usage for ports: isconnected(<PORT>, <INDEX>). Usage for component: isconnected(<INSTANCE>, <INDEX>)");
        }

        mathMatrixNameExpressionSymbol.setNameToAccess(" ");

        String valueListString = "";
        for (MathMatrixAccessSymbol accessSymbol : symbols)
            MathFunctionFixer.fixMathFunctions(accessSymbol, (EMAMBluePrintCPP) bluePrint);




//        valueListString += ExecuteMethodGenerator.generateExecuteCode(mathExpressionSymbol, new ArrayList<String>());

        String portinstanceName = ExecuteMethodGenerator.generateExecuteCode(symbols.get(0), new ArrayList<>());
        String indexCall = ExecuteMethodGenerator.generateExecuteCode(symbols.get(1), new ArrayList<>());

        if(!bluePrint.getVariable(portinstanceName).isPresent()){
            Log.error("Is_Connected("+portinstanceName+"...) : Can't find port "+portinstanceName+"!");
        }

        List<MathMatrixAccessSymbol> newMatrixAccessSymbols = new ArrayList<>();
        MathStringExpression stringExpression = new MathStringExpression("__"+portinstanceName+"_connected["+indexCall+"-1]",
                mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols());
        newMatrixAccessSymbols.add(new MathMatrixAccessSymbol(stringExpression));

        mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().setMathMatrixAccessSymbols(newMatrixAccessSymbols);
    }
}
