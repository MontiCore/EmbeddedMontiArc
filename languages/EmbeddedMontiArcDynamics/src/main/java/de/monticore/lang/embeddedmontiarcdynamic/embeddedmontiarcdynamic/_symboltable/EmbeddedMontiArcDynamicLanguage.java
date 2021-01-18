/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable;

import de.monticore.ast.ASTNode;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAConnectorResolvingFilter;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAConnectorSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortArraySymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstantiationSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAConnectorInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.cncModel.EMADynamicComponentSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.cncModel.EMADynamicEventHandlerSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.cncModel.EMADynamicPortArraySymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.*;
import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbol;
import de.monticore.lang.monticar.si._symboltable.SIUnitRangesSymbol;
import de.monticore.lang.monticar.si._symboltable.SIUnitSymbol;
import de.monticore.lang.monticar.ts.MCASTTypeSymbol;
import de.monticore.lang.monticar.ts.MCFieldSymbol;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.MontiCarTypeSymbol;
import de.monticore.lang.monticar.types2._symboltable.UnitNumberResolutionSymbol;
import de.monticore.modelloader.ModelingLanguageModelLoader;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.monticore.symboltable.resolving.CommonResolvingFilter;
import de.monticore.symboltable.types.JMethodSymbol;

import java.util.Optional;

//TODO: https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/languages/EmbeddedMontiArc/blob/master/src/main/java/de/monticore/lang/embeddedmontiarc/embeddedmontiarc/_symboltable/EmbeddedMontiArcModelLoader.java


public class EmbeddedMontiArcDynamicLanguage extends EmbeddedMontiArcDynamicLanguageTOP {

    public static final String FILE_ENDING = "emad";


    public EmbeddedMontiArcDynamicLanguage(){
        super("Embedded MontiArc Dynamic Language" , FILE_ENDING);
    }

    public EmbeddedMontiArcDynamicLanguage(String langName, String fileEnding) {
        super(langName, fileEnding);
    }

    @Override
    protected ModelingLanguageModelLoader<? extends ASTNode> provideModelLoader() {
        return new EmbeddedMontiArcDynamicModelLoader(this);
    }


    @Override
    protected void initResolvingFilters() {
        super.initResolvingFilters();

        //this.addResolvingFilter();

        //addResolvingFilter(CommonResolvingFilter.create(EMAComponentInstantiationSymbol.KIND));

        addResolvingFilter(CommonResolvingFilter.create(EMADynamicComponentSymbol.KIND));
        addResolvingFilter(CommonResolvingFilter.create(EMADynamicEventHandlerSymbol.KIND));
        addResolvingFilter(CommonResolvingFilter.create(EMADynamicEventHandlerInstanceSymbol.KIND));
//        addResolvingFilter(new EMAConnectorResolvingFilter(EMADynamicConnectorSymbol.KIND));
        addResolvingFilter(new EMAConnectorResolvingFilter(EMADynamicConnectorInstanceSymbol.KIND));



        this.addResolvingFilter(CommonResolvingFilter.create(EMAComponentInstanceSymbol.KIND));
        this.addResolvingFilter(CommonResolvingFilter.create(EMAComponentInstantiationSymbol.KIND));

        this.addResolvingFilter(CommonResolvingFilter.create(EMADynamicComponentInstanceSymbol.KIND));
        this.addResolvingFilter(CommonResolvingFilter.create(EMADynamicComponentInstantiationSymbol.KIND));

        //TODO : Add Dynamic Port symbol
        this.addResolvingFilter(CommonResolvingFilter.create(EMAPortSymbol.KIND));
        this.addResolvingFilter(CommonResolvingFilter.create(EMAPortArraySymbol.KIND));
        this.addResolvingFilter(CommonResolvingFilter.create(EMADynamicPortArraySymbol.KIND));
        this.addResolvingFilter(CommonResolvingFilter.create(EMAPortInstanceSymbol.KIND));
        this.addResolvingFilter(CommonResolvingFilter.create(EMADynamicPortInstanceSymbol.KIND));
//        this.addResolvingFilter(CommonResolvingFilter.create(EMADynamicPortInstanceSymbol.KIND));


        this.addResolvingFilter(new EMAConnectorResolvingFilter(EMAConnectorSymbol.KIND));
        this.addResolvingFilter(new EMAConnectorResolvingFilter(EMAConnectorInstanceSymbol.KIND));
        this.addResolvingFilter(CommonResolvingFilter.create(SIUnitSymbol.KIND));
        this.addResolvingFilter(CommonResolvingFilter.create(SIUnitRangesSymbol.KIND));
        this.addResolvingFilter(CommonResolvingFilter.create(MCTypeSymbol.KIND));
        this.addResolvingFilter(CommonResolvingFilter.create(MCFieldSymbol.KIND));
        this.addResolvingFilter(CommonResolvingFilter.create(MCASTTypeSymbol.KIND));
        this.addResolvingFilter(CommonResolvingFilter.create(JMethodSymbol.KIND));
        this.addResolvingFilter(CommonResolvingFilter.create(ResolutionDeclarationSymbol.KIND));
        this.addResolvingFilter(CommonResolvingFilter.create(UnitNumberResolutionSymbol.KIND));
        this.addResolvingFilter(CommonResolvingFilter.create(MontiCarTypeSymbol.KIND));


        this.setModelNameCalculator(new EmbeddedMontiArcDynamicModelNameCalculator());


        //addResolvingFilters(HOST_LANGUAGE.getResolvingFilters());

    }

    @Override
    public Optional<EmbeddedMontiArcDynamicSymbolTableCreator> getSymbolTableCreator(ResolvingConfiguration resolvingConfiguration, MutableScope enclosingScope) {
        //return super.getSymbolTableCreator(resolvingConfiguration, enclosingScope);
        return Optional.of(new EmbeddedMontiArcDynamicSymbolTableCreator(resolvingConfiguration, enclosingScope));
    }



}
