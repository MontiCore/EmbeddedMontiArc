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
import de.monticore.lang.math._symboltable.matrix.MathMatrixNameExpressionSymbol;

import java.util.HashSet;

/**
 * @author Sascha Schneiders.
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

    protected abstract void convert(MathExpressionSymbol mathExpressionSymbol, BluePrint bluePrint);

    public void convertAndSetTargetLanguageName(MathExpressionSymbol mathExpressionSymbol, BluePrint bluePrint) {
        convert(mathExpressionSymbol, bluePrint);
        if (mathExpressionSymbol instanceof MathMatrixNameExpressionSymbol) {
            MathMatrixNameExpressionSymbol mathMatrixNameExpressionSymbol = (MathMatrixNameExpressionSymbol) mathExpressionSymbol;
            targetLanguageCommandNames.add(mathMatrixNameExpressionSymbol.getTextualRepresentation());
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
                if (s.contains(command))
                    return true;
        return false;
    }


}
