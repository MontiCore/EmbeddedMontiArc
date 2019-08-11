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
package de.monticore.lang.monticar.emadl.adapter;

import de.monticore.lang.monticar.cnnarch._symboltable.ParameterSymbol;
import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbol;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberResolution;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.resolving.TransitiveAdaptedResolvingFilter;

public class ResolutionDeclarationSymbol2ParameterSymbolTypeFilter extends TransitiveAdaptedResolvingFilter<ParameterSymbol> {

    public ResolutionDeclarationSymbol2ParameterSymbolTypeFilter() {
        super(ResolutionDeclarationSymbol.KIND,
                ParameterSymbol.class,
                ParameterSymbol.KIND);
    }

    @Override
    public Symbol translate(Symbol adaptee) {
        assert adaptee instanceof ResolutionDeclarationSymbol;
        if (((ResolutionDeclarationSymbol) adaptee).getASTResolution() instanceof ASTUnitNumberResolution){
            return new ResolutionDeclarationSymbol2ParameterSymbol((ResolutionDeclarationSymbol) adaptee, ((ASTUnitNumberResolution) ((ResolutionDeclarationSymbol) adaptee).getASTResolution()));
        }
        return null;
    }
}