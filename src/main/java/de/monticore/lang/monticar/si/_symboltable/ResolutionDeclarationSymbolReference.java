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
package de.monticore.lang.monticar.si._symboltable;

import de.monticore.lang.monticar.resolution._ast.ASTResolution;
import de.monticore.symboltable.modifiers.AccessModifier;
import de.monticore.symboltable.references.SymbolReference;

/**
 * Created by Sascha on 14.05.2017.
 */
public class ResolutionDeclarationSymbolReference extends ResolutionDeclarationSymbol implements SymbolReference<ResolutionDeclarationSymbol> {
    protected int dimension = 0;

    public ResolutionDeclarationSymbolReference(final String name, String nameToResolve, ASTResolution astResolution) {
        super(name, nameToResolve, astResolution);
    }

    public static ResolutionDeclarationSymbolReference constructResolutionDeclSymbolRef(String nameToResolve,
                                                                                        ASTResolution astResolution) {

        return new ResolutionDeclarationSymbolReference("ResolutionDeclarationSymbol", nameToResolve, astResolution);
    }

    @Override
    public ResolutionDeclarationSymbol getReferencedSymbol() {
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
}
