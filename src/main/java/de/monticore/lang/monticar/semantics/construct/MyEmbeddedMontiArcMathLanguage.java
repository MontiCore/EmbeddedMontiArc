/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.construct;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.EmbeddedMontiArcMathLanguage;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.EmbeddedMontiArcMathSymbolTableCreator;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;

import java.util.Optional;
import java.util.Set;

public class MyEmbeddedMontiArcMathLanguage extends EmbeddedMontiArcMathLanguage {


    private final Set<ComponentReplacement> componentReplacements;

    public MyEmbeddedMontiArcMathLanguage(Set<ComponentReplacement> componentReplacements) {
        this.componentReplacements = componentReplacements;
    }

    @Override
    public Optional<EmbeddedMontiArcMathSymbolTableCreator> getSymbolTableCreator(ResolvingConfiguration resolvingConfiguration, MutableScope enclosingScope) {
        // TODO: forward replacement parameters to SymbolTableCreator
        MyEmbeddedMontiArcSymbolTableCreator symbolTableCreator = new MyEmbeddedMontiArcSymbolTableCreator(resolvingConfiguration, enclosingScope);
        symbolTableCreator.setComponentReplacements(componentReplacements);
        return Optional.of(symbolTableCreator);
    }
}
