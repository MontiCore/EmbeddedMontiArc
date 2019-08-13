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

import de.monticore.ast.ASTNode;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchSimpleExpressionSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ParameterSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ParameterType;
import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbol;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberResolution;
import de.monticore.symboltable.resolving.SymbolAdapter;
import de.se_rwth.commons.SourcePosition;
import org.jscience.mathematics.number.Rational;

import java.util.Optional;

public class ResolutionDeclarationSymbol2ParameterSymbol extends ParameterSymbol
        implements SymbolAdapter<ResolutionDeclarationSymbol> {

    private final ResolutionDeclarationSymbol adaptee;

    public ResolutionDeclarationSymbol2ParameterSymbol(ResolutionDeclarationSymbol ps, ASTUnitNumberResolution unitNumberResolution) {
        super(ps.getName());
        setType(ParameterType.ARCHITECTURE_PARAMETER);
        Double doubleValue = unitNumberResolution.getNumberWithUnit().getNumber().get();
        setExpression(ArchSimpleExpressionSymbol.of((doubleValue % 1)!= 0
                ? doubleValue.intValue()
                : doubleValue));
        this.adaptee = ps;
    }

    @Override
    public ResolutionDeclarationSymbol getAdaptee() {
        return adaptee;
    }

    @Override
    public Optional<ASTNode> getAstNode() {
        return getAdaptee().getAstNode();
    }

    @Override
    public SourcePosition getSourcePosition() {
        return getAdaptee().getSourcePosition();
    }

}