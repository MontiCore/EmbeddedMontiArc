/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable;

import com.google.common.collect.ImmutableSet;
import de.monticore.EmbeddingModelingLanguage;
import de.monticore.ModelingLanguage;
import de.monticore.antlr4.MCConcreteParser;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcLanguage;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._parser.EmbeddedMontiArcMathParser;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath.adapter.PortArraySymbol2MathVariableDeclarationSymbolTypeFilter;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath.adapter.PortSymbol2MathVariableDeclarationTypeFilter;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath.adapter.ResolutionDeclarationSymbol2MathVariableDeclarationTypeFilter;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.EmbeddedMontiArcDynamicLanguage;
import de.monticore.lang.math._symboltable.MathLanguage;
import de.monticore.lang.mathopt._symboltable.MathOptLanguage;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.resolving.ResolvingFilter;

import java.util.*;

/**
 * Created by MichaelvonWenckstern on 02.02.2017.
 */
public class EmbeddedMontiArcMathLanguage extends EmbeddingModelingLanguage{
    public static final String FILE_ENDING = "emam";
    public static final ModelingLanguage HOST_LANGUAGE =
            new EmbeddedMontiArcDynamicLanguage();
    public static final ModelingLanguage EMBEDDED_LANGUAGE =
            new MathOptLanguage();


    public EmbeddedMontiArcMathLanguage() {
        super("Embedded MontiArc (+dynamics) Math (+Opt) Language", FILE_ENDING,
                HOST_LANGUAGE, EMBEDDED_LANGUAGE);
    }

    @Override
    public Collection<ResolvingFilter<? extends Symbol>> getResolvingFilters() {
        List<ResolvingFilter<? extends Symbol>> ret =
                new ArrayList<>(super.getResolvingFilters());

        ret.add(new ResolutionDeclarationSymbol2MathVariableDeclarationTypeFilter());
        ret.add(new PortArraySymbol2MathVariableDeclarationSymbolTypeFilter());

        // ret.add(new PortSymbol2MathVariableDeclarationTypeFilter());
        return ret;
    }



    @Override
    protected EmbeddedMontiArcMathModelLoader provideModelLoader() {
        return new EmbeddedMontiArcMathModelLoader(this);
    }

    @Override
    public MCConcreteParser getParser() {
        return new EmbeddedMontiArcMathParser();
    }

    @Override
    public Optional<EmbeddedMontiArcMathSymbolTableCreator> getSymbolTableCreator(ResolvingConfiguration resolvingConfiguration, MutableScope enclosingScope) {
        return Optional.of(new EmbeddedMontiArcMathSymbolTableCreator(
                resolvingConfiguration, enclosingScope));
    }

}
