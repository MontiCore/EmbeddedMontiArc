/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.converter;


import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAConnectorInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicPortInstanceSymbol;
import de.monticore.lang.math._ast.ASTAssignmentType;
import de.monticore.lang.monticar.common2._ast.ASTCommonMatrixType;
import de.monticore.lang.monticar.generator.Variable;
import de.monticore.lang.monticar.generator.cpp.EMAMBluePrintCPP;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.references.MCASTTypeSymbolReference;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.types.types._ast.ASTType;
import de.se_rwth.commons.logging.Log;

import java.util.Optional;

/**
 */
public class PortConverter {
    public static int counterConstantPortsIn = 0;
    public static int counterConstantPortsOut = 0;

    public static int counterConstantPorts = 0;

    public static Variable getVariableForPortSymbol(EMAConnectorInstanceSymbol connectorSymbol, String connectName, EMAMBluePrintCPP bluePrint) {
        /*Optional<Variable> variable = bluePrint.getVariable(PortSymbol.getNameWithoutArrayBracketPart(connectName));
        if (variable.isPresent())
            return variable.get();*/
        return convertPortSymbolToVariable(connectorSymbol, connectName, bluePrint);
    }

    public static Variable convertPortSymbolToVariable(EMAConnectorInstanceSymbol connectorSymbol, String connectName, EMAMBluePrintCPP bluePrint) {
    /*    Variable variable = bluePrint.getVariable(PortConverter.getPortNameWithoutArrayBracketPart(connectName)).orElse(null);
        if (variable != null&&variable.isArray())
            return variable;
*/
        EMAPortInstanceSymbol portSymbol;
        if (connectName.equals(connectorSymbol.getSource())) {
            portSymbol = connectorSymbol.getSourcePort();
        } else
            portSymbol = connectorSymbol.getTargetPort();


        return convertPortSymbolToVariable(portSymbol, connectName, bluePrint);
    }

    public static Variable convertPortSymbolToVariable(EMAPortInstanceSymbol portSymbol, String connectName, EMAMBluePrintCPP bluePrint) {
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
        variable.addAdditionalInformation(portSymbol.isIncoming() ? Variable.INCOMING : Variable.OUTGOING);
        bluePrint.getMathInformationRegister().addVariable(variable);

        if(portSymbol instanceof EMADynamicPortInstanceSymbol){
            variable.setDynamic(((EMADynamicPortInstanceSymbol) portSymbol).isDynamic());
//            variable.setConstantValue("true");
//            variable.setIsConstantVariable(true);
        }

        Log.debug("EMAVAR: " + variable.getName() + " targetType:" + variable.getVariableType().getTypeNameTargetLanguage() + " isArray:" + variable.isArray(), "PortConverter");

        return variable;
    }

//    public static Variable convertPortNameToVariable(String portName, EMAComponentInstanceSymbol instance, EMAMBluePrintCPP bluePrintCPP){
//        String fullName = portName;
//        if(portName.contains(".")){
//            instance = instance.getSubComponent(portName.substring(0, portName.indexOf("."))).orElse(null);
//
//            portName = portName.substring(portName.indexOf(".")+1);
//        }
//        if(instance == null){
//            Log.error("Can't find instance for port: "+portName);
//            return null;
//        }
//
//        Optional<EMAPortInstanceSymbol> port = instance.getPortInstance(portName);
//        if(!port.isPresent()){
//            Log.error("Can't find port: "+portName);
//            return null;
//        }
//
//        return convertPortSymbolToVariable(port.get(), fullName, bluePrintCPP);
//
//    }

    private static void handlePortDirection(EMAPortInstanceSymbol portSymbol, Variable variable) {
        if (portSymbol.isIncoming()) {
            variable.setInputVariable(true);
        } else {
            variable.setInputVariable(false);
        }

    }

    private static String handlePortName(EMAPortInstanceSymbol portSymbol, Variable variable, String connectName) {
        String name = "";
        if (portSymbol.isConstant()) {
            //name += "Constant" + ++counterConstantPorts;
            name += connectName;
            variable.setIsConstantVariable(true);

            //OLD: variable.setConstantValue(((ConstantPortSymbol) portSymbol).getConstantValue().getValueAsString());
            //NEW:
            if(portSymbol.getConstantValue().isPresent()){
                variable.setConstantValue(portSymbol.getConstantValue().get().getValueAsString());
            }

            // Log.error("0xCOPOSHNOBECRASAAVA Constant Port should not be created as a variable");
        } else {
            //Log.info(portSymbol.getName(),"PORTNAME:");
            name += connectName;
        }
        return name;
    }

    private static void addVariableProperties(EMAPortInstanceSymbol portSymbol, Variable variable) {
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

    private static Optional<ASTType> getAstTypeFromPortSymbol(EMAPortInstanceSymbol portSymbol) {
        Optional<ASTType> result = Optional.empty();
        MCTypeReference<? extends MCTypeSymbol> typeRef = portSymbol.getTypeReference();
        if (typeRef.existsReferencedSymbol() && typeRef.getReferencedSymbol() instanceof MCASTTypeSymbolReference) {
            MCASTTypeSymbolReference typeSymbolReference = (MCASTTypeSymbolReference) portSymbol.getTypeReference().getReferencedSymbol();
            ASTType astType = typeSymbolReference.getAstType();
            result = Optional.of(astType);
        }
        return result;
    }

    public static Optional<ASTCommonMatrixType> getCommonMatrixTypeFromPortSymbol(EMAPortInstanceSymbol portSymbol) {
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
