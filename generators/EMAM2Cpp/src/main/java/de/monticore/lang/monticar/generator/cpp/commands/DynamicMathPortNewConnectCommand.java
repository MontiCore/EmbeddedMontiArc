/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.commands;

import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixAccessSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixNameExpressionSymbol;
import de.monticore.lang.monticar.generator.*;
import de.monticore.lang.monticar.generator.cpp.EMAMBluePrintCPP;
import de.monticore.lang.monticar.generator.cpp.MathFunctionFixer;
import de.monticore.lang.monticar.generator.cpp.converter.ExecuteMethodGenerator;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.MathStringExpression;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class DynamicMathPortNewConnectCommand extends MathCommand {

    protected static int ports_connect_functionID = 0;

    public DynamicMathPortNewConnectCommand() {
        setMathCommandName("ports_connect");
    }

    @Override
    protected void convert(MathExpressionSymbol mathExpressionSymbol, EMAMBluePrint bluePrint) {
        ports_connect_functionID++;

        MathMatrixNameExpressionSymbol mathMatrixNameExpressionSymbol = (MathMatrixNameExpressionSymbol) mathExpressionSymbol;

        List<MathMatrixAccessSymbol> symbols = mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols();
        if(symbols.size() % 3 != 0){
//            Log.error("Is connected wrong number of arguments! Usage for ports: isconnected(<PORT>, <INDEX>). Usage for component: isconnected(<INSTANCE>, <INDEX>)");
            Log.error("Ports_Connect wrong number of arguments! Usage: ports_connect(port, out portID, initialValue) or ports_connect(a, out aID, aInit, b, out bID, bInit,...)");

        }

        mathMatrixNameExpressionSymbol.setNameToAccess(" ");


        for (MathMatrixAccessSymbol accessSymbol : symbols)
            MathFunctionFixer.fixMathFunctions(accessSymbol, (EMAMBluePrintCPP) bluePrint);

        String call = "__ports_connect_"+ports_connect_functionID;
        Method connM = new Method();
        connM.setName(call);
        connM.setReturnTypeName("bool");
        connM.setPublic(false);
        call += "(";

        int n = 0;
        String check = "";
        String initSetter = "";
        for(int i = 0; i < symbols.size(); i += 3){
            String portName = ExecuteMethodGenerator.generateExecuteCode(symbols.get(i), new ArrayList<>());
            String portResultId = ExecuteMethodGenerator.generateExecuteCode(symbols.get(i+1), new ArrayList<>());
            String portInit = ExecuteMethodGenerator.generateExecuteCode(symbols.get(i+2), new ArrayList<>());

            Variable var = new Variable();
            var.setName("id"+n);
            var.setTypeNameTargetLanguage("int*");
            connM.addParameter(var);

            var = new Variable();
            var.setName("init"+n);
            Optional<Variable> portVar = bluePrint.getVariable(portName);
            if(portVar.isPresent()) {
                var.setVariableType(portVar.get().getVariableType());
            }else{
                Log.error("ports_connect: Can't find dynamic port "+portName);
            }
            connM.addParameter(var);

            call += "&"+portResultId+", "+portInit;
            if( (i+3) < symbols.size()){
                call+=", ";
            }

            connM.addInstruction(new TargetCodeInstruction(String.format("*id%d = dynamicconnect(%d, __%s_connected, &__%s_connect_request);\n",
                    n, portVar.get().getArraySize(), portName, portName)));
            connM.addInstruction(new TargetCodeInstruction(String.format(" __%s_free_request.push(*id%d);\n", portName,n)));

            if(i > 0){
                check += " || ";
            }
            check += "(*id"+n+" < 0)";

            initSetter += portName+"[*id"+n+"] = init"+n+"; *id"+n+" = (*id"+n+")+1;\n";


        }

        connM.addInstruction(new TargetCodeInstruction("if("+check+"){ return false; }\n"));
        connM.addInstruction(new TargetCodeInstruction(initSetter));
        connM.addInstruction(new TargetCodeInstruction("if(__parent != NULL){__parent_dynamic(__parent, true, false);}\n"));
        connM.addInstruction(new TargetCodeInstruction("return true;\n"));


        bluePrint.addMethod(connM);

        List<MathMatrixAccessSymbol> newMatrixAccessSymbols = new ArrayList<>();
        MathStringExpression stringExpression = new MathStringExpression(call+")",
                mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols());
        newMatrixAccessSymbols.add(new MathMatrixAccessSymbol(stringExpression));

        mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().setMathMatrixAccessSymbols(newMatrixAccessSymbols);
    }
}
