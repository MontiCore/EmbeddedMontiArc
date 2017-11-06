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
package de.monticore.lang.monticar.types2._symboltable;

import de.monticore.lang.monticar.types2._ast.ASTUnitNumberResolution;
import de.monticore.symboltable.modifiers.AccessModifier;
import de.monticore.symboltable.types.references.ActualTypeArgument;
import de.monticore.lang.monticar.ts.references.MCTypeReference;

import java.util.List;

/**
 * @author Sascha
 */
public class UnitNumberResolutionSymbolReference extends UnitNumberResolutionSymbol implements MCTypeReference<UnitNumberResolutionSymbol> {
    protected int dimension = 0;

    public UnitNumberResolutionSymbolReference(final String name, ASTUnitNumberResolution astUnitNumber) {
        super(name, astUnitNumber);
    }

    @Override
    public UnitNumberResolutionSymbol getReferencedSymbol() {
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
        return dimension;
    }

    @Override
    public void setDimension(int dimension) {
        this.dimension = dimension;
    }

    @Override
    public List<ActualTypeArgument> getActualTypeArguments() {
        return null;
    }

    @Override
    public void setActualTypeArguments(List<ActualTypeArgument> list) {
    }

}
