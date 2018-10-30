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
package de.monticore.lang.monticar.generator.cpp.converter;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ConnectorSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ConstantPortSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.math._ast.ASTAssignmentType;
import de.monticore.lang.monticar.common2._ast.ASTCommonMatrixType;
import de.monticore.lang.monticar.generator.Variable;
import de.monticore.lang.monticar.generator.cpp.BluePrintCPP;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.references.MCASTTypeSymbolReference;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.types.types._ast.ASTType;
import de.se_rwth.commons.logging.Log;

import java.util.Optional;

/**
 * @author Sascha Schneiders
 */
public class PortConverter {
    public static int counterConstantPortsIn = 0;
    public static int counterConstantPortsOut = 0;

    public static int counterConstantPorts = 0;

    public static Variable getVariableForPortSymbol(ConnectorSymbol connectorSymbol, String connectName, BluePrintCPP bluePrint) {
        /*Optional<Variable> variable = bluePrint.getVariable(PortSymbol.getNameWithoutArrayBracketPart(connectName));
        if (variable.isPresent())
            return variable.get();*/
        return convertPortSymbolToVariable(connectorSymbol, connectName, bluePrint);
    }

    public static Variable convertPortSymbolToVariable(ConnectorSymbol connectorSymbol, String connectName, BluePrintCPP bluePrint) {
    /*    Variable variable = bluePrint.getVariable(PortConverter.getPortNameWithoutArrayBracketPart(connectName)).orElse(null);
        if (variable != null&&variable.isArray())
            return variable;
*/
        PortSymbol portSymbol;
        if (connectName.equals(connectorSymbol.getSource())) {
            portSymbol = connectorSymbol.getSourcePort();
        } else
            portSymbol = connectorSymbol.getTargetPort();


        return convertPortSymbolToVariable(portSymbol, connectName, bluePrint);
    }

    public static Variable convertPortSymbolToVariable(PortSymbol portSymbol, String connectName, BluePrintCPP bluePrint) {
    /*    Variable variable = bluePrint.getVariable(PortConverter.getPortNameWithoutArrayBracketPart(connectName)).orElse(null);
        if (variable != null&&variable.isArray())
            return variable;
*/
        Variable variable = new Variable();
        String name = "";

        String typeNameMontiCar = portSymbol.getTypeReference().getName();

        addVariableProperties(portSymbol, variable);
        handlePortDirection(portSymbol, variable);
        name += handlePortName(portSymbol, variable, connectName);

        variable.setName(name);
        variable.setVariableType(TypeConverter.getVariableTypeForMontiCarTypeName(typeNameMontiCar, variable, portSymbol).get());
        variable.addAdditionalInformation(Variable.ORIGINPORT);
        bluePrint.getMathInformationRegister().addVariable(variable);
        Log.debug("EMAVAR: " + variable.getName() + " targetType:" + variable.getVariableType().getTypeNameTargetLanguage() + " isArray:" + variable.isArray(), "PortConverter");

        return variable;
    }

    private static void handlePortDirection(PortSymbol portSymbol, Variable variable) {
        if (portSymbol.isIncoming()) {
            variable.setInputVariable(true);
        } else {
            variable.setInputVariable(false);
        }

    }

    private static String handlePortName(PortSymbol portSymbol, Variable variable, String connectName) {
        String name = "";
        if (portSymbol.isConstant()) {
            //name += "Constant" + ++counterConstantPorts;
            name += connectName;
            variable.setIsConstantVariable(true);
            variable.setConstantValue(((ConstantPortSymbol) portSymbol).getConstantValue().getValueAsString());

            // Log.error("0xCOPOSHNOBECRASAAVA Constant Port should not be created as a variable");
        } else {
            //Log.info(portSymbol.getName(),"PORTNAME:");
            name += connectName;
        }
        return name;
    }

    private static void addVariableProperties(PortSymbol portSymbol, Variable variable) {
        MCTypeReference<? extends MCTypeSymbol> typeRef = portSymbol.getTypeReference();
        if (typeRef.existsReferencedSymbol() && typeRef.getReferencedSymbol() instanceof MCASTTypeSymbolReference) {
            MCASTTypeSymbolReference typeSymbolReference = (MCASTTypeSymbolReference) portSymbol.getTypeReference().getReferencedSymbol();
            ASTType astType = typeSymbolReference.getAstType();
            if (astType instanceof ASTAssignmentType) {
                ASTAssignmentType astAssignmentType = (ASTAssignmentType) astType;
                //if (astAssignmentType.getMatrixProperty().size() > 0) Log.error(astType.toString());
                variable.addProperties(astAssignmentType.getMatrixPropertyList());
            }
        }
    }

    private static Optional<ASTType> getAstTypeFromPortSymbol(PortSymbol portSymbol) {
        Optional<ASTType> result = Optional.empty();
        MCTypeReference<? extends MCTypeSymbol> typeRef = portSymbol.getTypeReference();
        if (typeRef.existsReferencedSymbol() && typeRef.getReferencedSymbol() instanceof MCASTTypeSymbolReference) {
            MCASTTypeSymbolReference typeSymbolReference = (MCASTTypeSymbolReference) portSymbol.getTypeReference().getReferencedSymbol();
            ASTType astType = typeSymbolReference.getAstType();
            result = Optional.of(astType);
        }
        return result;
    }

    public static Optional<ASTCommonMatrixType> getCommonMatrixTypeFromPortSymbol(PortSymbol portSymbol) {
        Optional<ASTCommonMatrixType> result = Optional.empty();
        Optional<ASTType> astType = getAstTypeFromPortSymbol(portSymbol);
        if (astType.isPresent()) {
            if (astType.get() instanceof ASTCommonMatrixType) {
                result = Optional.of((ASTCommonMatrixType) astType.get());
            }
        }
        return result;
    }

    public static String getPortNameWithoutArrayBracketPart(String name) {
        String nameWithOutArrayBracketPart = name;
        if (nameWithOutArrayBracketPart.endsWith("]")) {
            char lastChar;
            do {
                lastChar = nameWithOutArrayBracketPart.charAt(nameWithOutArrayBracketPart.length() - 1);
                nameWithOutArrayBracketPart = nameWithOutArrayBracketPart.substring(0, nameWithOutArrayBracketPart.length() - 1);
            } while (lastChar != '[');
        }
        return nameWithOutArrayBracketPart;
    }
}
