/*
 * Copyright (c) 2017, MontiCore. All rights reserved.
 *
 * http://www.se-rwth.de/
 */
package de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable;

import com.google.common.collect.ImmutableSet;
import de.monticore.ast.ASTNode;
import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbol;
import de.monticore.lang.monticar.types2._symboltable.UnitNumberResolutionSymbol;
import de.monticore.lang.monticar.si._symboltable.SIUnitRangesSymbol;
import de.monticore.lang.monticar.si._symboltable.SIUnitSymbol;
import de.monticore.lang.montiarc.tagging._symboltable.TagSymbolCreator;
import de.monticore.lang.montiarc.tagging._symboltable.TagableModelingLanguage;
import de.monticore.modelloader.ModelingLanguageModelLoader;
import de.monticore.symboltable.resolving.CommonResolvingFilter;
import de.monticore.symboltable.types.JFieldSymbol;
import de.monticore.symboltable.types.JMethodSymbol;
import de.monticore.symboltable.types.JTypeSymbol;

import java.util.LinkedHashSet;

/**
 * The MontiArc Language
 *
 * @author Robert Heim, Michael von Wenckstern
 */
public class EmbeddedMontiViewLanguage extends EmbeddedMontiViewLanguageTOP implements TagableModelingLanguage {

    public static final String FILE_ENDING = "emv";

    protected LinkedHashSet<TagSymbolCreator> tagSymbolCreators = new LinkedHashSet<>();

    public EmbeddedMontiViewLanguage() {
        super("Embedded MontiView Language", FILE_ENDING);
    }

    @Override
    protected void initResolvingFilters() {
        super.initResolvingFilters();
        // is done in generated TOP-language addResolver(new
        // CommonResolvingFilter<ComponentSymbol>(ComponentSymbol.class, ComponentSymbol.KIND));
        addResolver(new CommonResolvingFilter<>(ComponentInstanceSymbol.KIND));
        addResolver(new CommonResolvingFilter<>(PortSymbol.KIND));
        addResolvingFilter(CommonResolvingFilter.create(PortArraySymbol.KIND));
        addResolver(new EMAConnectorResolvingFilter<>(ConnectorSymbol.KIND));
        addResolver(new EMAConnectorResolvingFilter<>(EffectorSymbol.KIND));
        addResolver(new CommonResolvingFilter<>(ExpandedComponentInstanceSymbol.KIND));
        addResolver(new CommonResolvingFilter<>(SIUnitSymbol.KIND));
        addResolver(new CommonResolvingFilter<>(SIUnitRangesSymbol.KIND));
        addResolver(new CommonResolvingFilter<>(JTypeSymbol.KIND));
        addResolver(new CommonResolvingFilter<>(JFieldSymbol.KIND));
        addResolver(new CommonResolvingFilter<>(JMethodSymbol.KIND));
        addResolvingFilter(CommonResolvingFilter.create(ResolutionDeclarationSymbol.KIND));
        addResolvingFilter(CommonResolvingFilter.create(UnitNumberResolutionSymbol.KIND));
        setModelNameCalculator(new EmbeddedMontiArcModelNameCalculator());
    }

    public void addTagSymbolCreator(TagSymbolCreator tagSymbolCreator) {
        this.tagSymbolCreators.add(tagSymbolCreator);
    }

    public ImmutableSet<TagSymbolCreator> getTagSymbolCreators() {
        return ImmutableSet.copyOf(this.tagSymbolCreators);
    }

    /**
     * @see de.monticore.CommonModelingLanguage#provideModelLoader()
     */
    @Override
    protected ModelingLanguageModelLoader<? extends ASTNode> provideModelLoader() {
        return new EmbeddedMontiViewModelLoader(this);
    }
}
