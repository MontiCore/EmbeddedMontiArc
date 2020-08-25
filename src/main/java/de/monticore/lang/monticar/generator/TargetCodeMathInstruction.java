/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator;

import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 */
public class TargetCodeMathInstruction implements Instruction {
    protected String targetCode;
    protected MathExpressionSymbol mathExpressionSymbol;
    Optional<String> addedVariable = Optional.empty();

    List<String> usesVariables = new ArrayList<>();

    public TargetCodeMathInstruction(String targetCode, MathExpressionSymbol mathExpressionSymbol) {
        this.targetCode = targetCode;
        this.mathExpressionSymbol = mathExpressionSymbol;
    }

    public MathExpressionSymbol getMathExpressionSymbol() {
        return mathExpressionSymbol;
    }

    public void setMathExpressionSymbol(MathExpressionSymbol mathExpressionSymbol) {
        this.mathExpressionSymbol = mathExpressionSymbol;
    }

    public String getTargetCode() {
        return targetCode;
    }

    public void setTargetCode(String targetCode) {
        this.targetCode = targetCode;
    }

    @Override
    public String getTargetLanguageInstruction() {
        return targetCode;
    }

    @Override
    public boolean isConnectInstruction() {
        return false;
    }

    @Override
    public boolean isTargetCodeInstruction() {
        return true;
    }


    public List<String> getUsedVariables() {
        return usesVariables;
    }

    public void setUsesVariables(List<String> usesVariables) {
        this.usesVariables = usesVariables;
    }

    public void addUsedVariable(String variableName) {
        usesVariables.add(variableName);
    }

    public Optional<String> getAddedVariable() {
        return addedVariable;
    }

    public void setAddedVariable(Optional<String> addedVariable) {
        this.addedVariable = addedVariable;
    }

    public void setAddedVariable(String variableName) {
        this.addedVariable = Optional.of(variableName);
    }
}
