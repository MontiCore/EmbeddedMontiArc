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

public class DynamicMathPortFreeCommand extends MathCommand {

    protected static int ports_free_functionID = 0;

    public DynamicMathPortFreeCommand() {
        setMathCommandName("ports_free");
    }

    @Override
    protected void convert(MathExpressionSymbol mathExpressionSymbol, EMAMBluePrint bluePrint) {
        ports_free_functionID++;


        MathMatrixNameExpressionSymbol mathMatrixNameExpressionSymbol = (MathMatrixNameExpressionSymbol) mathExpressionSymbol;

        List<MathMatrixAccessSymbol> symbols = mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols();
        if(symbols.size()%2 != 0){
            Log.error("Ports_Free wrong number of arguments! Usage: ports_free(portA, indexA, portB, indexB, .....) ");
        }

        mathMatrixNameExpressionSymbol.setNameToAccess(" ");

        for (MathMatrixAccessSymbol accessSymbol : symbols)
            MathFunctionFixer.fixMathFunctions(accessSymbol, (EMAMBluePrintCPP) bluePrint);


        String call = "__ports_free_"+ports_free_functionID;
        Method connM = new Method();
        connM.setName(call);
        connM.setReturnTypeName("bool");
        connM.setPublic(false);
        call += "(";
        TargetCodeInstruction tci = new TargetCodeInstruction("");
        connM.addInstruction(tci);


        int n = 0;
        String idxTest = "";
        for(int i = 0; i < symbols.size(); i += 2) {
            String portName = ExecuteMethodGenerator.generateExecuteCode(symbols.get(i), new ArrayList<>());
            String portIndex = ExecuteMethodGenerator.generateExecuteCode(symbols.get(i+1), new ArrayList<>());

            Optional<Variable> portVar = bluePrint.getVariable(portName);
            if(!portVar.isPresent()) {
                Log.error("ports_connect: Can't find dynamic port "+portName);
            }

            Variable var = new Variable();
            var.setName("idx"+n);
            var.setTypeNameTargetLanguage("int");
            connM.addParameter(var);


            connM.addInstruction(new TargetCodeInstruction("__"+portName+"_free_request.push(idx"+n+");\n"));
            //connM.addInstruction(new TargetCodeInstruction("__"+portName+"_connected[idx"+n+"] = false;\n"));


            idxTest += "(idx"+n+" < 0) || ("+portVar.get().getArraySize()+" <= idx"+n+") || (!__"+portName+"_connected[idx"+n+"])";

            call += "("+portIndex+")-1";
            if( (i+2) < symbols.size()){
                call+=", ";
                idxTest += " || ";
            }



            ++n;
        }

        tci.setInstruction("if("+idxTest+"){return false;}");

        connM.addInstruction(new TargetCodeInstruction("if(__parent != NULL){__parent_dynamic(__parent, false, true);}\n"));
        connM.addInstruction(new TargetCodeInstruction("return true;\n"));

        bluePrint.addMethod(connM);

        List<MathMatrixAccessSymbol> newMatrixAccessSymbols = new ArrayList<>();
        MathStringExpression stringExpression = new MathStringExpression(call+")",
                mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols());
        newMatrixAccessSymbols.add(new MathMatrixAccessSymbol(stringExpression));

        mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().setMathMatrixAccessSymbols(newMatrixAccessSymbols);
    }
}
