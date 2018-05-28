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
package de.monticore.lang.monticar.ts;

import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.lang.monticar.types2._ast.ASTType;
import de.monticore.symboltable.CommonSymbol;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.Scope;

import java.util.Collections;
import java.util.List;
import java.util.Optional;

/**
 * @author Sascha Schneiders
 */
public class MCASTTypeSymbol extends CommonSymbol implements MCTypeSymbol {
    public static final MCASTTypeSymbolKind KIND;

    static {
        KIND = MCASTTypeSymbolKind.INSTANCE;
    }

    protected ASTType astType = null;

    public MCASTTypeSymbol(String name) {
        super(name, KIND);
    }

    public MCASTTypeSymbol(String name, ASTType astType) {
        super(name, KIND);
        this.astType = astType;
    }

    public MCASTTypeSymbol(String name, MutableScope enclosingScope, ASTType astType) {
        super(name, KIND);
        setEnclosingScope(enclosingScope);
        this.astType = astType;
    }

    public ASTType getAstType() {
        return astType;
    }

    public List<? extends MCTypeSymbol> getFormalTypeParameters() {
        return Collections.emptyList();
    }

    public Optional<? extends MCTypeReference<? extends MCTypeSymbol>> getSuperClass() {
        return Optional.empty();
    }

    public List<? extends MCTypeReference<? extends MCTypeSymbol>> getInterfaces() {
        return Collections.emptyList();
    }

    public List<? extends MCTypeReference<? extends MCTypeSymbol>> getSuperTypes() {
        return Collections.emptyList();
    }

    public boolean isFormalTypeParameter() {
        return false;
    }

    public Scope getSpannedScope() {
        return null;
    }
}
