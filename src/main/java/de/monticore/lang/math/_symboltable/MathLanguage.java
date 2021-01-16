/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math._symboltable;

import de.monticore.lang.math.LogConfig;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.symboltable.resolving.CommonResolvingFilter;

/**
 */

public class MathLanguage extends MathLanguageTOP{
    /** known file ending from matlab "*.m" */
    public static final String FILE_ENDING = "m";

    public MathLanguage() {
        super("Math Language", FILE_ENDING);
    }

    @Override
    protected MathModelLoader provideModelLoader() {
        return new MathModelLoader(this);
    }

    @Override
    protected void initResolvingFilters() {
        super.initResolvingFilters();
        //addResolver(CommonResolvingFilter.create(AssignmentSymbol.KIND));
        //addResolver(CommonResolvingFilter.create(MathVariableDeclarationSymbol.KIND));
        addResolvingFilter(CommonResolvingFilter.create(MathExpressionSymbol.KIND));
        addResolvingFilter(CommonResolvingFilter.create(MathStatementsSymbol.KIND));

        setModelNameCalculator(new MathModelNameCalculator());
    }
}
