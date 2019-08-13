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
import de.monticore.expressionsbasis._ast.ASTExpression;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortArraySymbol;
import de.monticore.lang.math._ast.ASTNameExpression;
import de.monticore.lang.math._ast.ASTNumberExpression;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchSimpleExpressionSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchTypeSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.IODeclarationSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ParameterSymbol;
import de.monticore.lang.monticar.common2._ast.ASTCommonMatrixType;
import de.monticore.lang.monticar.ts.MCASTTypeSymbol;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.types2._ast.ASTElementType;
import de.monticore.types.types._ast.ASTType;
import de.monticore.symboltable.resolving.SymbolAdapter;
import de.se_rwth.commons.SourcePosition;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class PortArraySymbol2IODeclarationSymbol extends IODeclarationSymbol
        implements SymbolAdapter<EMAPortArraySymbol> {

    private final EMAPortArraySymbol adaptee;

    public PortArraySymbol2IODeclarationSymbol(EMAPortArraySymbol ps) {
        super(ps.getName());
        setInput(ps.isIncoming());
        setArrayLength(ps.getDimension());

        ArchTypeSymbol type = new ArchTypeSymbol();
        List<ArchSimpleExpressionSymbol> shape = getShape(ps);
        if (shape.size() >= 1){
            type.setChannelIndex(0);
        }
        if (shape.size() >= 2){
            type.setHeightIndex(1);
        }
        if (shape.size() >= 3){
            type.setWidthIndex(2);
        }
        type.setDomain(getElementType(ps));
        type.setDimensionSymbols(shape);

        setType(type);

        this.adaptee = ps;
    }

    @Override
    public EMAPortArraySymbol getAdaptee() {
        return adaptee;
    }


    private ASTType getASTType(EMAPortArraySymbol port){
        MCTypeSymbol type = port.getTypeReference().getReferencedSymbol();
        if (type instanceof MCASTTypeSymbol){
            return ((MCASTTypeSymbol) type).getAstType();
        }
        else {
            throw new IllegalStateException("Unknown port type");
        }
    }

    private List<ArchSimpleExpressionSymbol> getShape(EMAPortArraySymbol port){
        List<ArchSimpleExpressionSymbol> dimensionList = new ArrayList<>(4);
        ASTType astType = getASTType(port);

        if (astType instanceof ASTCommonMatrixType){
            ASTCommonMatrixType matrixType = (ASTCommonMatrixType) astType;
            for (ASTExpression element : matrixType.getDimension().getDimensionList()){
                if (element instanceof ASTNumberExpression){
                    int dimension = ((ASTNumberExpression) element).getNumberWithUnit().getNumber().get().intValue();
                    dimensionList.add(ArchSimpleExpressionSymbol.of(dimension));
                }
                else {
                    String instName = element instanceof ASTNameExpression ? ((ASTNameExpression) element).getName() : element.getSymbolOpt().get().getName();
                    ParameterSymbol variable= port.getEnclosingScope()
                            .<ParameterSymbol>resolve(instName, ParameterSymbol.KIND).get();
                    dimensionList.add(variable.getExpression());
                }
            }
        }

        return dimensionList;
    }

    private ASTElementType getElementType(EMAPortArraySymbol port){
        ASTType astType = getASTType(port);
        if (astType instanceof ASTCommonMatrixType){
            return  ((ASTCommonMatrixType) astType).getElementType();
        }
        else if (astType instanceof ASTElementType){
            return (ASTElementType) astType;
        }
        else {
            throw new IllegalStateException("Unknown port ast type");
        }
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