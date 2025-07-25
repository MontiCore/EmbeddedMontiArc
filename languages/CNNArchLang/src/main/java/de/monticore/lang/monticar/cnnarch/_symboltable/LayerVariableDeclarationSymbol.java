/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */

package de.monticore.lang.monticar.cnnarch._symboltable;

import de.monticore.symboltable.CommonSymbol;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.Symbol;

import java.util.Collection;

public class LayerVariableDeclarationSymbol extends VariableDeclarationSymbol {

    private LayerSymbol layer;

    protected LayerVariableDeclarationSymbol(String name) {
        super(name);
    }

    public LayerSymbol getLayer() {
        return layer;
    }

    protected void setLayer(LayerSymbol layer) {
        this.layer = layer;
    }

    @Override
    public void putInScope(Scope scope) {
        Collection<Symbol> symbolsInScope = scope.getLocalSymbols().get(getName());
        if (symbolsInScope == null || !symbolsInScope.contains(this)) {
            scope.getAsMutableScope().add(this);
            layer.putInScope(scope);
        }
    }

    @Override
    public VariableDeclarationSymbol preResolveDeepCopy() {
        LayerVariableDeclarationSymbol copy = new LayerVariableDeclarationSymbol(getName());

        if (getAstNode().isPresent()) {
            copy.setAstNode(getAstNode().get());
        }

        copy.setLayer((LayerSymbol) getLayer().preResolveDeepCopy());

        return copy;
    }

}
