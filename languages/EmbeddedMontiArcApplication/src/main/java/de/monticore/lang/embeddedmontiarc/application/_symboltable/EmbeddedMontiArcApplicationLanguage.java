/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.application._symboltable;

import de.monticore.EmbeddingModelingLanguage;
import de.monticore.ModelingLanguage;
import de.monticore.antlr4.MCConcreteParser;
import de.monticore.lang.application.application._symboltable.ApplicationLanguage;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcLanguage;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcapplication._parser.EmbeddedMontiArcApplicationParser;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.resolving.ResolvingFilter;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Optional;

/**
 */
public class EmbeddedMontiArcApplicationLanguage extends EmbeddingModelingLanguage{
    public static final String FILE_ENDING = "emaapl";
    public static final ModelingLanguage HOST_LANGUAGE =
            new EmbeddedMontiArcLanguage();
    public static final ModelingLanguage EMBEDDED_LANGUAGE =
            new ApplicationLanguage();


    public EmbeddedMontiArcApplicationLanguage() {
        super("Embedded MontiArc Math Language", FILE_ENDING,
                HOST_LANGUAGE, EMBEDDED_LANGUAGE);
    }

    @Override
    public Collection<ResolvingFilter<? extends Symbol>> getResolvingFilters() {
        List<ResolvingFilter<? extends Symbol>> ret =
                new ArrayList<>(super.getResolvingFilters());
        //ret.add(new ResolutionDeclarationSymbol2MathVariableDeclarationTypeFilter());
        //ret.add(new PortArraySymbol2MathVariableDeclarationSymbolTypeFilter());
        // ret.add(new PortSymbol2MathVariableDeclarationTypeFilter());
        return ret;
    }

    @Override
    protected EmbeddedMontiArcApplicationModelLoader provideModelLoader() {
        return new EmbeddedMontiArcApplicationModelLoader(this);
    }

    @Override
    public MCConcreteParser getParser() {
        return new EmbeddedMontiArcApplicationParser();
    }

    @Override
    public Optional<EmbeddedMontiArcApplicationSymbolTableCreator> getSymbolTableCreator(ResolvingConfiguration resolvingConfiguration, MutableScope enclosingScope) {
        return Optional.of(new EmbeddedMontiArcApplicationSymbolTableCreator(
                resolvingConfiguration, enclosingScope));
    }

}
