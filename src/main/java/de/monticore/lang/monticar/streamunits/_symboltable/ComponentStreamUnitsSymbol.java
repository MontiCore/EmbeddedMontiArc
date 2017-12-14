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
