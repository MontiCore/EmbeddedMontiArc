/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.ide._symboltable;

import de.monticore.lang.monticar.sol.grammars.artifact._symboltable.ArtifactSymbol;
import de.monticore.lang.monticar.sol.grammars.artifact._symboltable.ArtifactSymbolReference;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.references.SymbolReference;

import java.util.Optional;
import java.util.function.Function;

public class WriteSymbol extends WriteSymbolTOP {
    protected String relativePath;

    public WriteSymbol(String name) {
        super(name);
    }

    public void setRelativePath(String relativePath) {
        this.relativePath = relativePath;
    }

    public String getRelativePath() {
        return Optional.ofNullable(this.relativePath).orElse(".");
    }

    protected <T extends Symbol> SymbolReference<T> getSymbolReference(Function<String, SymbolReference<T>> constructor) {
        return constructor.apply(this.getName());
    }

    public boolean isReferencingArtifact() {
        Scope enclosingScope = this.getEnclosingScope();
        SymbolReference<ArtifactSymbol> reference = this.getSymbolReference(name -> new ArtifactSymbolReference(name, enclosingScope));

        return reference.existsReferencedSymbol();
    }

    public boolean isReferencingModule() {
        Scope enclosingScope = this.getEnclosingScope();
        SymbolReference<ModuleSymbol> reference = this.getSymbolReference(name -> new ModuleSymbolReference(name, enclosingScope));

        return reference.existsReferencedSymbol();
    }

    public Optional<ArtifactSymbol> getReferencedArtifact() {
        Scope enclosingScope = this.getEnclosingScope();
        SymbolReference<ArtifactSymbol> reference = this.getSymbolReference(name -> new ArtifactSymbolReference(name, enclosingScope));

        return reference.existsReferencedSymbol() ? Optional.of(reference.getReferencedSymbol()) : Optional.empty();
    }
}
