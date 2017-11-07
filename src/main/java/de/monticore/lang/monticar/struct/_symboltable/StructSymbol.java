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
package de.monticore.lang.monticar.struct._symboltable;

import de.monticore.lang.monticar.ts.CommonMCTypeSymbol;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.symboltable.Symbols;

import java.util.Collection;

public class StructSymbol extends CommonMCTypeSymbol<MCTypeSymbol, MCTypeReference<MCTypeSymbol>> {

    public static final StructKind KIND = new StructKind();

    public StructSymbol(String name) {
        super(name, KIND);
    }

    public Collection<StructFieldDefinitionSymbol> getStructFieldDefinitions() {
        return Symbols.sortSymbolsByPosition(getSpannedScope().resolveLocally(StructFieldDefinitionSymbol.KIND));
    }
}
