/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnntrain._symboltable;

import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.resolving.CommonResolvingFilter;

public class CNNTrainLanguage extends CNNTrainLanguageTOP {

    public static final String FILE_ENDING = "cnnt";

    public CNNTrainLanguage() {
        super("CNNTrain Language", FILE_ENDING);
    }

    @Override
    protected CNNTrainModelLoader provideModelLoader() {
        return new CNNTrainModelLoader(this);
    }

    @Override
    protected void initResolvingFilters() {
        super.initResolvingFilters();
        addResolvingFilter(new CNNTrainCompilationUnitResolvingFilter());
        addResolvingFilter(new CommonResolvingFilter<Symbol>(ConfigurationSymbol.KIND));
        addResolvingFilter(new CommonResolvingFilter<Symbol>(OptimizerSymbol.KIND));
        addResolvingFilter(new CommonResolvingFilter<Symbol>(OptimizerParamSymbol.KIND));
        setModelNameCalculator(new CNNTrainModelNameCalculator());
    }

}
