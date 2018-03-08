package de.monticore.lang.monticar.generator.roscpp.instructions;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.monticar.common2._ast.ASTCommonMatrixType;
import de.monticore.lang.monticar.generator.TargetCodeInstruction;
import de.monticore.lang.monticar.generator.roscpp.helper.IndexHelper;
import de.monticore.lang.monticar.generator.roscpp.helper.NameHelper;
import de.monticore.lang.monticar.generator.rosmsg.RosMsg;
import de.monticore.lang.monticar.ts.MCASTTypeSymbol;

import java.util.List;
import java.util.stream.Collectors;

public class SetStructPortInstruction extends TargetCodeInstruction {
    public SetStructPortInstruction(PortSymbol port, RosMsg rosMsg) {
        if (rosMsg.getName().startsWith("std_msgs/")) {
            if (rosMsg.getName().endsWith("MultiArray")) {
                ASTCommonMatrixType matrixType = (ASTCommonMatrixType) ((MCASTTypeSymbol) port.getTypeReference().getReferencedSymbol()).getAstType();
                List<String> dimSizes = IndexHelper.getDimSizesOfMatrixType(matrixType);

                this.instruction = "";
                this.instruction += "int counter = 0;\n";
                String indexString = "";
                for (int i = 0; i < dimSizes.size(); i++) {
                    String curInd = "i" + i;
                    indexString += (i == 0 ? "" : ", ") + curInd;
                    this.instruction += "for(int " + curInd + " = 0; " + curInd + " < " + dimSizes.get(i) + "; " + curInd + "++){\n";
                }

                this.instruction += "(component->" + NameHelper.getPortNameTargetLanguage(port) + ")(" + indexString + ") = msg->data[counter]";
                //TODO: check type not name
                if (rosMsg.getName().equals("std_msgs/ByteMultiArray")) {
                    //is a bool msg
                    this.instruction += " != 0";
                }
                this.instruction += ";\n";
                this.instruction += "counter++;\n";

                for (int i = 0; i < dimSizes.size(); i++) {
                    this.instruction += "}\n";
                }


            } else {
                this.instruction = NameHelper.getAllFieldNames(rosMsg).stream()
                        .map(field -> "component->" + NameHelper.getPortNameTargetLanguage(port) + " = msg->" + field + ";")
                        .sorted()
                        .collect(Collectors.joining("\n"));
            }
        } else {
            this.instruction = NameHelper.getAllFieldNames(rosMsg).stream()
                    .map(field -> "component->" + NameHelper.getPortNameTargetLanguage(port) + "." + field + " = msg->" + field + ";")
                    .sorted()
                    .collect(Collectors.joining("\n"));
        }
    }

}
