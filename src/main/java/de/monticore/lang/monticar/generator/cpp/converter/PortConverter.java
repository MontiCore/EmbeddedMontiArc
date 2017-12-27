package de.monticore.lang.monticar.generator.cpp.converter;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ConnectorSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ConstantPortSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.math.math._ast.ASTAssignmentType;
import de.monticore.lang.monticar.generator.Variable;
import de.monticore.lang.monticar.generator.cpp.BluePrintCPP;
import de.monticore.lang.monticar.ts.references.MCASTTypeSymbolReference;
import de.monticore.lang.monticar.types2._ast.ASTType;

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

        if (portSymbol.getTypeReference().getReferencedSymbol() instanceof MCASTTypeSymbolReference) {
            MCASTTypeSymbolReference typeSymbolReference = (MCASTTypeSymbolReference) portSymbol.getTypeReference().getReferencedSymbol();
            ASTType astType = typeSymbolReference.getAstType();
            ASTAssignmentType astAssignmentType = (ASTAssignmentType) astType;
            //if (astAssignmentType.getMatrixProperty().size() > 0) Log.error(astType.toString());
            variable.addProperties(astAssignmentType.getMatrixProperty());
        }
        if (portSymbol.isIncoming())

        {
            variable.setInputVariable(true);
        } else

        {
            variable.setInputVariable(false);
        }

        if (portSymbol.isConstant())

        {
            //name += "Constant" + ++counterConstantPorts;
            name += connectName;
            variable.setIsConstantVariable(true);
            variable.setConstantValue(((ConstantPortSymbol) portSymbol).getConstantValue().getValueAsString());

            // Log.error("0xCOPOSHNOBECRASAAVA Constant Port should not be created as a variable");
        } else

        {
            //Log.info(portSymbol.getName(),"PORTNAME:");
            name += connectName;
        }

        variable.setName(name);
        variable.setVariableType(TypeConverter.getVariableTypeForMontiCarTypeName(typeNameMontiCar, variable, portSymbol).get());
        variable.addAdditionalInformation(Variable.ORIGINPORT);
        bluePrint.getMathInformationRegister().addVariable(variable);

        return variable;
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
