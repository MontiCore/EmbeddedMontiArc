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

import de.monticore.lang.monticar.ts.MontiCarTypeSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortArraySymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.math.math._symboltable.MathVariableDeclarationSymbol;
import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbol;
import de.monticore.lang.monticar.si._symboltable.SIUnitRangesSymbol;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.resolving.TransitiveAdaptedResolvingFilter;

/**
 * Created by Sascha on 15.08.2017.
 */
public class PortSymbol2MathVariableDeclarationTypeFilter extends TransitiveAdaptedResolvingFilter<MathVariableDeclarationSymbol> {

    public PortSymbol2MathVariableDeclarationTypeFilter() {
        super(PortSymbol.KIND,
                MathVariableDeclarationSymbol.class,
                MathVariableDeclarationSymbol.KIND);
    }

    @Override
    public Symbol translate(Symbol adaptee) {
        assert adaptee instanceof PortSymbol;
        if (((PortSymbol) adaptee).getTypeReference().getReferencedSymbol() instanceof SIUnitRangesSymbol)
            return new PortSymbol2MathVariableDeclarationSymbol((PortSymbol) adaptee, (SIUnitRangesSymbol) ((PortSymbol) adaptee).getTypeReference().getReferencedSymbol());
        else {
            MontiCarTypeSymbol jTypeSymbol = (MontiCarTypeSymbol) ((PortSymbol) adaptee).getTypeReference().getReferencedSymbol();
            System.out.println("Adaption of: " + jTypeSymbol.toString());
            return new PortSymbol2MathVariableDeclarationSymbol((PortSymbol) adaptee);
        }

    }
}
