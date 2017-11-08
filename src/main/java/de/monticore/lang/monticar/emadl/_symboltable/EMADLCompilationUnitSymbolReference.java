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
package de.monticore.lang.monticar.emadl._symboltable;

import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.modifiers.AccessModifier;
import de.monticore.symboltable.references.CommonSymbolReference;
import de.monticore.symboltable.references.SymbolReference;

public class EMADLCompilationUnitSymbolReference extends EMADLCompilationUnitSymbol implements SymbolReference<EMADLCompilationUnitSymbol> {
    protected final SymbolReference<EMADLCompilationUnitSymbol> reference;

    public EMADLCompilationUnitSymbolReference(final String name, final Scope enclosingScopeOfReference) {
        super(name);
        reference = new CommonSymbolReference<>(name, EMADLCompilationUnitSymbol.KIND, enclosingScopeOfReference);
    }


    /*
   * Methods of SymbolReference interface
   */


    @Override
    public EMADLCompilationUnitSymbol getReferencedSymbol() {
        return reference.getReferencedSymbol();
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
        return getReferencedSymbol().getName();
    }

    @Override
    public String getFullName() {
        return getReferencedSymbol().getFullName();
    }

    @Override
    public void setEnclosingScope(MutableScope scope) {
        getReferencedSymbol().setEnclosingScope(scope);
    }

    @Override
    public Scope getEnclosingScope() {
        return getReferencedSymbol().getEnclosingScope();
    }

    @Override
    public AccessModifier getAccessModifier() {
        return getReferencedSymbol().getAccessModifier();
    }

    @Override
    public void setAccessModifier(AccessModifier accessModifier) {
        getReferencedSymbol().setAccessModifier(accessModifier);
    }


}
