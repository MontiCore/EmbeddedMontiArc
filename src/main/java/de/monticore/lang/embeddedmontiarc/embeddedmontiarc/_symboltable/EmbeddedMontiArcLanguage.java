/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable;

import de.monticore.ast.ASTNode;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAConnectorResolvingFilter;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAConnectorSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortArraySymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstantiationSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAConnectorInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbol;
import de.monticore.lang.monticar.si._symboltable.SIUnitRangesSymbol;
import de.monticore.lang.monticar.si._symboltable.SIUnitSymbol;
import de.monticore.lang.monticar.ts.MCASTTypeSymbol;
import de.monticore.lang.monticar.ts.MCFieldSymbol;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.MontiCarTypeSymbol;
import de.monticore.lang.monticar.types2._symboltable.UnitNumberResolutionSymbol;
import de.monticore.modelloader.ModelingLanguageModelLoader;
import de.monticore.symboltable.resolving.CommonResolvingFilter;
import de.monticore.symboltable.types.JMethodSymbol;

/**
 * The MontiArc Language
 *
 */
public class EmbeddedMontiArcLanguage extends EmbeddedMontiArcLanguageTOP {

    public static final String FILE_ENDING = "ema";


    public EmbeddedMontiArcLanguage() {
        super("Embedded MontiArc Language", FILE_ENDING);
    }

    @Override
    protected void initResolvingFilters() {
        super.initResolvingFilters();
        // is done in generated TOP-language addResolver(new
        // CommonResolvingFilter<EMAComponentSymbol>(EMAComponentSymbol.class, EMAComponentSymbol.KIND));
        addResolvingFilter(CommonResolvingFilter.create(EMAComponentInstantiationSymbol.KIND));
        addResolvingFilter(CommonResolvingFilter.create(EMAPortSymbol.KIND));
        addResolvingFilter(CommonResolvingFilter.create(EMAPortArraySymbol.KIND));
        addResolvingFilter(new EMAConnectorResolvingFilter<>(EMAConnectorSymbol.KIND));
        addResolvingFilter(new EMAConnectorResolvingFilter<>(EMAConnectorInstanceSymbol.KIND));
        addResolvingFilter(CommonResolvingFilter.create(EMAComponentInstanceSymbol.KIND));
        addResolvingFilter(CommonResolvingFilter.create(SIUnitSymbol.KIND));
        addResolvingFilter(CommonResolvingFilter.create(SIUnitRangesSymbol.KIND));
        addResolvingFilter(CommonResolvingFilter.create(MCTypeSymbol.KIND));
        addResolvingFilter(CommonResolvingFilter.create(MCFieldSymbol.KIND));
        addResolvingFilter(CommonResolvingFilter.create(MCASTTypeSymbol.KIND));
        addResolvingFilter(CommonResolvingFilter.create(JMethodSymbol.KIND));
        addResolvingFilter(CommonResolvingFilter.create(ResolutionDeclarationSymbol.KIND));
        addResolvingFilter(CommonResolvingFilter.create(UnitNumberResolutionSymbol.KIND));
        //addResolvingFilter(CommonResolvingFilter.create(ComponentKind.KIND));
        //addResolvingFilter(CommonResolvingFilter.create(TagSymbol.KIND));
        addResolvingFilter(CommonResolvingFilter.create(MontiCarTypeSymbol.KIND));
        addResolvingFilter(CommonResolvingFilter.create(EMAPortInstanceSymbol.KIND));
        setModelNameCalculator(new EmbeddedMontiArcModelNameCalculator());
    }

    /**
     * @see de.monticore.CommonModelingLanguage#provideModelLoader()
     */
    @Override
    protected ModelingLanguageModelLoader<? extends ASTNode> provideModelLoader() {
        return new EmbeddedMontiArcModelLoader(this);
    }
}
