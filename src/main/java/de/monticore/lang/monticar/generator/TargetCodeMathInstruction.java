/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.monticore.lang.monticar.generator;

import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * @author Sascha Schneiders
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
