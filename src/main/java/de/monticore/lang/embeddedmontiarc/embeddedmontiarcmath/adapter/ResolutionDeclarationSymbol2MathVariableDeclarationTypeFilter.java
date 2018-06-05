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
package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath.adapter;

import de.monticore.lang.math._symboltable.MathVariableDeclarationSymbol;
import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbol;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberResolution;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.resolving.TransitiveAdaptedResolvingFilter;
import de.se_rwth.commons.logging.Log;

/**
 *  @author Sascha Schneiders
 */
public class ResolutionDeclarationSymbol2MathVariableDeclarationTypeFilter extends TransitiveAdaptedResolvingFilter<MathVariableDeclarationSymbol> {


    public ResolutionDeclarationSymbol2MathVariableDeclarationTypeFilter() {
        super(ResolutionDeclarationSymbol.KIND,
                MathVariableDeclarationSymbol.class,
                MathVariableDeclarationSymbol.KIND);
    }

    @Override
    public Symbol translate(Symbol adaptee) {
        if(adaptee instanceof  ResolutionDeclarationSymbol) {
            assert adaptee instanceof ResolutionDeclarationSymbol;
            if (((ResolutionDeclarationSymbol) adaptee).getASTResolution() instanceof ASTUnitNumberResolution)
                return new ResolutionDeclarationSymbol2MathVariableDeclarationSymbol((ResolutionDeclarationSymbol) adaptee, ((ASTUnitNumberResolution) ((ResolutionDeclarationSymbol) adaptee).getASTResolution()));
        }else{

            Log.debug(adaptee.toString(),"ResolutionDeclarationSymbol Adaption");
        }
        return null;
    }
}
