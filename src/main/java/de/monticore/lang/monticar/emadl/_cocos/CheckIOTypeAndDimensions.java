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
package de.monticore.lang.monticar.emadl._cocos;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortArraySymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.IODeclarationSymbol;
import de.monticore.lang.monticar.cnnarch.helper.Utils;
import de.monticore.lang.monticar.common2._ast.ASTCommonDimensionElement;
import de.monticore.lang.monticar.common2._ast.ASTCommonMatrixType;
import de.monticore.lang.monticar.emadl._ast.ASTArchIOPort;
import de.monticore.lang.monticar.emadl._symboltable.ArchPortConnectorSymbol;
import de.monticore.lang.monticar.emadl.helper.ErrorCodes;
import de.monticore.lang.monticar.ts.MCASTTypeSymbol;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.types2._ast.ASTElementType;
import de.monticore.lang.monticar.types2._ast.ASTType;
import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;

public class CheckIOTypeAndDimensions implements EMADLASTArchIOPortCoCo {

    @Override
    public void check(ASTArchIOPort node) {
        ArchPortConnectorSymbol sym = (ArchPortConnectorSymbol) node.getSymbol().get();
        PortArraySymbol port = sym.getPort().get();
        IODeclarationSymbol ioDeclaration = sym.getIoDeclaration().get();


        //check Type
        ASTElementType portType = getType(port);
        ASTElementType archType = getType(ioDeclaration);
        boolean typesAreEqual = Utils.equals(portType, archType);
        if (!typesAreEqual){
            Log.error("0" + ErrorCodes.INVALID_IO_TYPE_CODE + " Invalid port type. " +
                            "The type of port '" + node.getName() +
                            "' and the connected architecture IO declaration do not match. "
                    , port.getSourcePosition());
        }


        //check dimensions (including arrayLength)
        List<Integer> portDimensions = getDimensions(port);
        List<Integer> archDimensions = getDimensions(ioDeclaration);

        if (portDimensions.size() == archDimensions.size()){
            for (int i = 0; i < portDimensions.size(); i++){
                if (!portDimensions.get(i).equals(archDimensions.get(i))){
                    dimensionError(portDimensions, archDimensions, node);
                }
            }
        }
        else {
            dimensionError(portDimensions, archDimensions, node);
        }

    }

    private void dimensionError(List<Integer> portDimensions, List<Integer> archDimensions, ASTArchIOPort node){
        Log.error("0" + ErrorCodes.INVALID_IO_DIMENSIONS_CODE + " " +
                        "Port and architecture IO dimensions of '" + node.getName() + "' do not match. " +
                        "Embedded MontiArc port dimensions: " + Joiners.COMMA.join(portDimensions) + ". " +
                        "CNNArch IODeclaration dimensions: " + Joiners.COMMA.join(archDimensions) + ". "
                , node.get_SourcePositionStart());
    }

    private ASTType getASTType(PortArraySymbol port){
        MCTypeSymbol type = port.getTypeReference().getReferencedSymbol();
        if (type instanceof MCASTTypeSymbol){
            return ((MCASTTypeSymbol) type).getAstType();
        }
        else {
            throw new IllegalStateException("Unknown port type (see CheckIOTypeAndDimensions)");
        }
    }

    private List<Integer> getDimensions(PortArraySymbol port){
        List<Integer> dimensionList = new ArrayList<>(4);
        ASTType astType = getASTType(port);

        if (port.getDimension() != 1) {
            dimensionList.add(port.getDimension());
        }

        if (astType instanceof ASTCommonMatrixType){
            ASTCommonMatrixType matrixType = (ASTCommonMatrixType) astType;
            for (ASTCommonDimensionElement element : matrixType.getCommonDimension().getCommonDimensionElements()){
                int dimension = element.getUnitNumber().get().getNumber().get().getDividend().intValue();
                dimensionList.add(dimension);
            }
        }

        return dimensionList;
    }

    private ASTElementType getType(PortArraySymbol port){
        ASTType astType = getASTType(port);
        if (astType instanceof ASTCommonMatrixType){
            return  ((ASTCommonMatrixType) astType).getElementType();
        }
        else if (astType instanceof ASTElementType){
            return (ASTElementType) astType;
        }
        else {
            throw new IllegalStateException("Unknown port ast type (see CheckIOTypeAndDimensions)");
        }
    }

    private List<Integer> getDimensions(IODeclarationSymbol ioDeclaration){
        List<Integer> dimensionList = new ArrayList<>(4);

        if (ioDeclaration.getArrayLength() != 1){
            dimensionList.add(ioDeclaration.getArrayLength());
        }

        dimensionList.addAll(ioDeclaration.getType().getDimensions());

        return dimensionList;
    }

    private ASTElementType getType(IODeclarationSymbol ioDeclaration){
        return ioDeclaration.getType().getElementType();
    }
}
