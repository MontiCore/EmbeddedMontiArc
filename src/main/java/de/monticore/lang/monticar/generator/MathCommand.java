/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator;


import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixNameExpressionSymbol;

import java.util.HashSet;

import static de.monticore.lang.monticar.generator.cpp.MathCommandRegisterCPP.removeBrackets;

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

}
