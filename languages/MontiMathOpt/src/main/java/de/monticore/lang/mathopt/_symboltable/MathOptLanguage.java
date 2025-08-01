/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.mathopt._symboltable;

import de.monticore.lang.math.LogConfig;
import de.monticore.lang.math._symboltable.MathLanguage;
import de.monticore.lang.math._symboltable.MathModelNameCalculator;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.resolving.CommonResolvingFilter;
import de.monticore.symboltable.resolving.ResolvingFilter;

import java.util.Collection;
import java.util.Optional;

/**
 */
public class MathOptLanguage extends MathOptLanguageTOP {

    private MathLanguage parentLanguage;

    public MathOptLanguage() {
        super("Math Optimization Language", MathLanguage.FILE_ENDING);
        parentLanguage = new MathLanguage();
    }

    @Override
    protected MathOptModelLoader provideModelLoader() {
        return new MathOptModelLoader(this);
    }

    @Override
    public Optional<MathOptSymbolTableCreator> getSymbolTableCreator(ResolvingConfiguration resolvingConfiguration, MutableScope mutableScope) {
        return Optional.of(new MathOptSymbolTableCreator(resolvingConfiguration, mutableScope));
    }

    @Override
    protected void initResolvingFilters() {
        super.initResolvingFilters();
        addResolvingFilter(CommonResolvingFilter.create(MathExpressionSymbol.KIND));
        addResolvingFilter(CommonResolvingFilter.create(MathStatementsSymbol.KIND));

        setModelNameCalculator(new MathModelNameCalculator());
    }
}
