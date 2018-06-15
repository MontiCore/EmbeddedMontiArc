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

import de.monticore.lang.math._symboltable.JSValue;
import de.monticore.lang.math._symboltable.MathVariableDeclarationSymbol;
import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbol;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberResolution;
import de.monticore.symboltable.resolving.SymbolAdapter;

import java.util.Arrays;
import java.util.Collections;

import static de.monticore.numberunit.Rationals.doubleToRational;

/**
 * @author Sascha Schneiders
 */
public class ResolutionDeclarationSymbol2MathVariableDeclarationSymbol extends MathVariableDeclarationSymbol
        implements SymbolAdapter<ResolutionDeclarationSymbol> {

    private final ResolutionDeclarationSymbol adaptee;

    public ResolutionDeclarationSymbol2MathVariableDeclarationSymbol(ResolutionDeclarationSymbol ps, ASTUnitNumberResolution unitNumberResolution) {
        super(ps.getName(),
                new JSValue(doubleToRational(unitNumberResolution.getNumber().get())));

        this.adaptee = ps;
    }

    @Override
    public ResolutionDeclarationSymbol getAdaptee() {
        return adaptee;
    }

}
