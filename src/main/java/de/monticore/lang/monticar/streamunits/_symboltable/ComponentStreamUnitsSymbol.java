/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.streamunits._symboltable;

import de.monticore.lang.monticar.streamunits._ast.ASTComponentStreamUnits;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.SymbolKind;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class ComponentStreamUnitsSymbol extends ComponentStreamUnitsSymbolTOP {

    public ComponentStreamUnitsSymbol(String name) {
        super(name);
    }

    public Optional<NamedStreamUnitsSymbol> getNamedStream(String name) {
        return getSpannedScope().resolveLocally(name, NamedStreamUnitsSymbol.KIND);
    }

    public List<NamedStreamUnitsSymbol> getNamedStreams() {
        return new ArrayList<>(getSpannedScope().<NamedStreamUnitsSymbol>resolveLocally(NamedStreamUnitsSymbol.KIND));
    }

    public <T extends Symbol> Optional<T> getComponentSymbol(SymbolKind cmpKind) {
        return getEnclosingScope().resolve(getComponentName(), cmpKind);
    }

    public String getComponentName() {
        if (getAstNode().isPresent()) {
            ASTComponentStreamUnits node = (ASTComponentStreamUnits) getAstNode().get();
            return node.getComponentName();
        }
        return "";
    }
}
