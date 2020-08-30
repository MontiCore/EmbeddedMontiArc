/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.resolve;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.EmbeddedMontiArcMathLanguage;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.EmbeddedMontiArcMathSymbolTableCreator;
import de.monticore.lang.monticar.semantics.construct.Replacement;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;

import java.util.Optional;

public class MyEmbeddedMontiArcMathLanguage extends EmbeddedMontiArcMathLanguage {


    private final Replacement componentReplacements;

    public MyEmbeddedMontiArcMathLanguage(Replacement componentReplacements) {
        this.componentReplacements = componentReplacements;
    }

    @Override
    public Optional<EmbeddedMontiArcMathSymbolTableCreator> getSymbolTableCreator(ResolvingConfiguration resolvingConfiguration, MutableScope enclosingScope) {
        // TODO: forward replacement parameters to SymbolTableCreator
        MyEmbeddedMontiArcSymbolTableCreator symbolTableCreator = new MyEmbeddedMontiArcSymbolTableCreator(resolvingConfiguration, enclosingScope);
        symbolTableCreator.setReplacements(componentReplacements);
        return Optional.of(symbolTableCreator);
    }
}
