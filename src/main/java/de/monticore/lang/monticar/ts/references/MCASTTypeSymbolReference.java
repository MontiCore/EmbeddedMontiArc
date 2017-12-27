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
package de.monticore.lang.monticar.ts.references;

import de.monticore.lang.monticar.ts.MCASTTypeSymbol;
import de.monticore.lang.monticar.types2._ast.ASTType;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.modifiers.AccessModifier;
import de.monticore.symboltable.types.references.ActualTypeArgument;

import java.util.Collections;
import java.util.List;

/**
 * @author Sascha Schneiders
 */
public class MCASTTypeSymbolReference extends MCASTTypeSymbol implements MCTypeReference<MCASTTypeSymbol> {
    public MCASTTypeSymbolReference(String name) {
        super(name);
    }

    public MCASTTypeSymbolReference(String name, ASTType astType) {
        super(name, astType);
    }

    public MCASTTypeSymbolReference(String name, MutableScope enclosingScope, ASTType astType) {
        super(name, enclosingScope, astType);
    }

    public MCASTTypeSymbol getReferencedSymbol() {
        return this;
    }

    @Override
    public boolean existsReferencedSymbol() {
        return true;
    }

    @Override
    public boolean isReferencedSymbolLoaded() {
        return true;
    }

    @Override
    public AccessModifier getAccessModifier() {
        return this.getReferencedSymbol().getAccessModifier();
    }

    @Override
    public void setAccessModifier(AccessModifier accessModifier) {
        this.getReferencedSymbol().setAccessModifier(accessModifier);
    }

    @Override
    public int getDimension() {
        return 0;
    }

    @Override
    public void setDimension(int dimension) {

    }

    @Override
    public List<ActualTypeArgument> getActualTypeArguments() {
        return Collections.emptyList();
    }

    @Override
    public void setActualTypeArguments(List<ActualTypeArgument> list) {
    }

    public static MCASTTypeSymbolReference constructReference(String name, MutableScope enclosingScope, ASTType astType) {
        MCASTTypeSymbolReference astTypeSymbolRef = new MCASTTypeSymbolReference(name, enclosingScope, astType);
        return astTypeSymbolRef;
    }
}
