/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch._symboltable;


import de.monticore.symboltable.resolving.CommonResolvingFilter;

public class CNNArchLanguage extends CNNArchLanguageTOP {

    public static final String FILE_ENDING = "cnna";

    public CNNArchLanguage() {
        super("CNNArch Language", FILE_ENDING);
    }

    @Override
    protected CNNArchModelLoader provideModelLoader() {
        return new CNNArchModelLoader(this);
    }

    @Override
    protected void initResolvingFilters() {
        super.initResolvingFilters();
        //addResolvingFilter(CommonResolvingFilter.create(MathExpressionSymbol.KIND));
//        addResolvingFilter(new CNNArchCompilationUnitResolvingFilter()); // this is already added due to the call to super.initResolvingFilters()
        addResolvingFilter(CommonResolvingFilter.create(ArchitectureSymbol.KIND));
        addResolvingFilter(CommonResolvingFilter.create(LayerDeclarationSymbol.KIND));
        addResolvingFilter(CommonResolvingFilter.create(UnrollDeclarationSymbol.KIND));
        addResolvingFilter(CommonResolvingFilter.create(ArchitectureElementSymbol.KIND));
        addResolvingFilter(CommonResolvingFilter.create(ParameterSymbol.KIND));
        addResolvingFilter(CommonResolvingFilter.create(IODeclarationSymbol.KIND));
        addResolvingFilter(CommonResolvingFilter.create(ArgumentSymbol.KIND));
        //addResolvingFilter(CommonResolvingFilter.create(ArchExpressionSymbol.KIND));
        setModelNameCalculator(new CNNArchModelNameCalculator());
    }

}
