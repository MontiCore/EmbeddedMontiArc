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
package de.monticore.lang.math._symboltable;

import de.monticore.symboltable.Scope;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.modifiers.AccessModifier;
import de.monticore.symboltable.references.CommonSymbolReference;
import de.monticore.symboltable.references.SymbolReference;

/**
 * @author math-group
 */
public class MathVariableDeclarationSymbolReference extends MathVariableDeclarationSymbol implements SymbolReference<MathVariableDeclarationSymbol> {
    protected final SymbolReference<MathVariableDeclarationSymbol> reference;

    public MathVariableDeclarationSymbolReference(final String name, final Scope enclosingScopeOfReference) {
        super(name, null, null, null);
        reference = new CommonSymbolReference<>(name, KIND, enclosingScopeOfReference);
    }

  /*
   * Methods of SymbolReference interface
   */

    @Override
    public MathVariableDeclarationSymbol getReferencedSymbol() {

        try {
            return reference.getReferencedSymbol();
        } catch (Exception ex) {
            return null;

        }
    }

    @Override
    public boolean existsReferencedSymbol() {
        return reference.existsReferencedSymbol();
    }

    @Override
    public boolean isReferencedSymbolLoaded() {
        return reference.isReferencedSymbolLoaded();
    }

  /*
  * Methods of Symbol interface
  */

    @Override
    public String getName() {
        if (getReferencedSymbol() != null)
            return getReferencedSymbol().getName();
        return super.getName();
    }

    @Override
    public String getFullName() {
        if (getReferencedSymbol() != null) {
            return getReferencedSymbol().getFullName();
        }
        return super.getFullName();
    }

    @Override
    public void setEnclosingScope(MutableScope scope) {
        if (getReferencedSymbol() != null)
            getReferencedSymbol().setEnclosingScope(scope);
        super.setEnclosingScope(scope);
    }

    @Override
    public Scope getEnclosingScope() {

        if (getReferencedSymbol() != null)
            return getReferencedSymbol().getEnclosingScope();
        return super.getEnclosingScope();
    }

    @Override
    public AccessModifier getAccessModifier() {

        if (getReferencedSymbol() != null)
            return getReferencedSymbol().getAccessModifier();
        return super.getAccessModifier();
    }

    @Override
    public void setAccessModifier(AccessModifier accessModifier) {
        if (getReferencedSymbol() != null)
            getReferencedSymbol().setAccessModifier(accessModifier);
        super.setAccessModifier(accessModifier);
    }
}
