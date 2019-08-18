/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */

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
