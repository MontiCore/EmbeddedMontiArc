/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator;


import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixNameExpressionSymbol;
import de.monticore.lang.monticar.generator.cpp.EMAMBluePrintCPP;

import java.util.HashSet;
import java.util.List;
import java.util.Optional;

import static de.monticore.lang.monticar.generator.cpp.MathCommandRegisterCPP.removeBrackets;

/**
 */
public abstract class MathCommand {
    protected String mathCommandName;

    private HashSet<String> targetLanguageCommandNames = new HashSet<>();

    public MathCommand() {

    }

    public MathCommand(String mathCommandName) {
        this.mathCommandName = mathCommandName;
    }

    public String getMathCommandName() {
        return mathCommandName;
    }

    public void setMathCommandName(String mathCommandName) {
        this.mathCommandName = mathCommandName;
    }

    protected abstract void convert(MathExpressionSymbol mathExpressionSymbol, EMAMBluePrint bluePrint);

    public void convertAndSetTargetLanguageName(MathExpressionSymbol mathExpressionSymbol, EMAMBluePrint bluePrint) {
        convert(mathExpressionSymbol, bluePrint);
        if (mathExpressionSymbol instanceof MathMatrixNameExpressionSymbol) {
            MathMatrixNameExpressionSymbol mathMatrixNameExpressionSymbol = (MathMatrixNameExpressionSymbol) mathExpressionSymbol;
            String s = mathMatrixNameExpressionSymbol.getTextualRepresentation();
            s = removeBrackets(s);
            targetLanguageCommandNames.add(s);
        }
    }

    /**
     * Gets the mathCommandName converted to the target language possibly contains multiple
     * commands
     *
     * @return targetLanguageCommandName
     */
    protected HashSet<String> getTargetLanguageCommandNames() {
        return targetLanguageCommandNames;
    }

    public boolean isTargetLanguageCommand(String command) {
        if (!command.isEmpty())
            for (String s : getTargetLanguageCommandNames())
                if (s.contentEquals(command))
                    return true;
        return false;
    }

    public String getTypeOfFirstInput(MathMatrixNameExpressionSymbol mathMatrixNameExpressionSymbol, EMAMBluePrintCPP bluePrintCPP){
        String nameOfFirstParameter = mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols().get(0).getTextualRepresentation();
        for(Variable var: bluePrintCPP.getVariables()){
            String varName = var.getName();
            if(varName.equals(nameOfFirstParameter)){
                VariableType varType = var.getVariableType();
                String typeName = varType.getTypeNameTargetLanguage();
                return typeName;
            }

        }
        return "";
    }

    public static void redefineArmaMat(EMAMBluePrintCPP bluePrint){
        List<Variable> vars= bluePrint.getVariables();
        for(Variable var : vars){
        VariableType varType = var.getVariableType();
        String targetName = varType.getTypeNameTargetLanguage();
        if(targetName.equals("mat")){
            Variable newVar = var;
            newVar.setTypeNameTargetLanguage("arma::Mat<unsigned char>");
            bluePrint.replaceVariable(var, newVar);
        }else if(targetName.equals("cube")){
            Variable newVarCube = var;
            newVarCube.setTypeNameTargetLanguage("Cube<unsigned char>");
            bluePrint.replaceVariable(var, newVarCube);
        }
        }
    }

    public static void redefineInit(EMAMBluePrintCPP bluePrint){
        Optional<Method> methodOpt = bluePrint.getMethod("init");
        Method initMethod = methodOpt.get();
        List<Instruction> instructs = initMethod.getInstructions();

        for(Instruction instruct : instructs){
            String code = instruct.getTargetLanguageInstruction();
            if(code.contains("mat(")){
                code =code.replace("mat(", "Mat<unsigned char>(");
            } else if(code.contains("cube(")){
                code = code.replace("cube(", "Cube<unsigned char>(");
            }
            ((TargetCodeInstruction)instruct).setInstruction(code);
        }
    }

    public boolean isArgumentNoReturnMathCommand() {
        return false;
    }

    public boolean isCVMathCommand(){
        return false;
    }
}
