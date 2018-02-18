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

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortArraySymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchTypeSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.IODeclarationSymbol;
import de.monticore.lang.monticar.common2._ast.ASTCommonDimensionElement;
import de.monticore.lang.monticar.common2._ast.ASTCommonMatrixType;
import de.monticore.lang.monticar.ts.MCASTTypeSymbol;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.types2._ast.ASTElementType;
import de.monticore.lang.monticar.types2._ast.ASTType;
import de.monticore.symboltable.resolving.SymbolAdapter;

import java.util.ArrayList;
import java.util.List;

public class PortArraySymbol2IODeclarationSymbol extends IODeclarationSymbol
        implements SymbolAdapter<PortArraySymbol> {

    private final PortArraySymbol adaptee;

    public PortArraySymbol2IODeclarationSymbol(PortArraySymbol ps) {
        super(ps.getName());
        setInput(ps.isIncoming());
        setArrayLength(ps.getDimension());

        ArchTypeSymbol type = new ArchTypeSymbol();
        List<Integer> shape = getShape(ps);
        if (shape.size() >= 1){
            type.setChannelIndex(0);
        }
        if (shape.size() >= 2){
            type.setHeightIndex(1);
        }
        if (shape.size() >= 3){
            type.setWidthIndex(2);
        }
        type.setElementType(getElementType(ps));
        type.setDimensions(shape);

        setType(type);

        this.adaptee = ps;
    }

    @Override
    public PortArraySymbol getAdaptee() {
        return adaptee;
    }


    private ASTType getASTType(PortArraySymbol port){
        MCTypeSymbol type = port.getTypeReference().getReferencedSymbol();
        if (type instanceof MCASTTypeSymbol){
            return ((MCASTTypeSymbol) type).getAstType();
        }
        else {
            throw new IllegalStateException("Unknown port type");
        }
    }

    private List<Integer> getShape(PortArraySymbol port){
        List<Integer> dimensionList = new ArrayList<>(4);
        ASTType astType = getASTType(port);

        if (astType instanceof ASTCommonMatrixType){
            ASTCommonMatrixType matrixType = (ASTCommonMatrixType) astType;
            for (ASTCommonDimensionElement element : matrixType.getCommonDimension().getCommonDimensionElements()){
                int dimension = element.getUnitNumber().get().getNumber().get().getDividend().intValue();
                dimensionList.add(dimension);
            }
        }

        return dimensionList;
    }

    private ASTElementType getElementType(PortArraySymbol port){
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

}