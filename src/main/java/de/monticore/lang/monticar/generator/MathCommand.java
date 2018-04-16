package de.monticore.lang.monticar.generator;


import de.monticore.lang.math.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math.math._symboltable.matrix.MathMatrixNameExpressionSymbol;

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
    public HashSet<String> getTargetLanguageCommandNames() {
        return targetLanguageCommandNames;
    }
}
