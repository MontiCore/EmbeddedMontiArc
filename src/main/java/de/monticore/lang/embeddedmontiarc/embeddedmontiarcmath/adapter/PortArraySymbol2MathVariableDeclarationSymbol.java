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

import java.util.Arrays;
import java.util.Collections;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortArraySymbol;
import de.monticore.lang.math._symboltable.MathVariableDeclarationSymbol;
import de.monticore.lang.monticar.ranges._ast.ASTRange;
import de.monticore.lang.monticar.si._symboltable.SIUnitRangesSymbol;
import de.monticore.symboltable.resolving.SymbolAdapter;

/**
 * Created by MichaelvonWenckstern on 27.01.2017.
 */
public class PortArraySymbol2MathVariableDeclarationSymbol extends MathVariableDeclarationSymbol
        implements SymbolAdapter<PortArraySymbol> {

    private final PortArraySymbol adaptee;

    public PortArraySymbol2MathVariableDeclarationSymbol(PortArraySymbol ps) {
        super(ps.getName(),
                new ASTRange(),
                Arrays.asList(1, ps.getDimension()),
                Collections.EMPTY_LIST);

        this.adaptee = ps;
    }

    public PortArraySymbol2MathVariableDeclarationSymbol(PortArraySymbol ps, SIUnitRangesSymbol rangesSymbol) {
        super(ps.getName(),
                rangesSymbol.getRange(0),
                Arrays.asList(1, ps.getDimension()),
                Collections.EMPTY_LIST);

        this.adaptee = ps;
    }

    @Override
    public PortArraySymbol getAdaptee() {
        return adaptee;
    }

}
