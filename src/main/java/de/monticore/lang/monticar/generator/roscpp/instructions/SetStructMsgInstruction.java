package de.monticore.lang.monticar.generator.roscpp.instructions;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.monticar.common2._ast.ASTCommonMatrixType;
import de.monticore.lang.monticar.generator.roscpp.util.TargetCodeInstruction;
import de.monticore.lang.monticar.generator.roscpp.helper.IndexHelper;
import de.monticore.lang.monticar.generator.roscpp.helper.NameHelper;
import de.monticore.lang.monticar.generator.rosmsg.RosMsg;
import de.monticore.lang.monticar.ts.MCASTTypeSymbol;

import java.util.List;
import java.util.stream.Collectors;

public class SetStructMsgInstruction extends TargetCodeInstruction {
    public SetStructMsgInstruction(EMAPortSymbol portSymbol, RosMsg rosMsg) {
        if (rosMsg.getName().startsWith("std_msgs/")) {
            if (rosMsg.getName().endsWith("MultiArray")) {
                ASTCommonMatrixType matrixType = (ASTCommonMatrixType) ((MCASTTypeSymbol) portSymbol.getTypeReference().getReferencedSymbol()).getAstType();
                List<String> dimSizes = IndexHelper.getDimSizesOfMatrixType(matrixType);

                String dataSize = "";
                for (int i = 0; i < dimSizes.size(); i++) {
                    dataSize += (i == 0 ? "" : " * ") + dimSizes.get(i);
                }

                this.instruction = "tmpMsg.data.resize(" + dataSize + ");\n";
                this.instruction += "int counter = 0;\n";
                String indexString = "";
                for (int i = 0; i < dimSizes.size(); i++) {
                    String curInd = "i" + i;
                    indexString += (i == 0 ? "" : ", ") + curInd;
                    this.instruction += "for(int " + curInd + " = 0; " + curInd + " < " + dimSizes.get(i) + "; " + curInd + "++){\n";
                }

                this.instruction += "tmpMsg.data[counter] = (component->" + NameHelper.getPortNameTargetLanguage(portSymbol) + ")(" + indexString + ")";
                    //TODO: check type not name
                    if (rosMsg.getName().equals("std_msgs/ByteMultiArray")) {
                        //is a bool msg
                        this.instruction += " ? 1 : 0";
                    }
                    this.instruction += ";\n";
                this.instruction += "counter++;\n";

                for (int i = 0; i < dimSizes.size(); i++) {
                    this.instruction += "}\n";
                }


            } else {
                this.instruction = NameHelper.getAllFieldNames(rosMsg).stream()
                        .map(field -> "tmpMsg." + field + " = component->" + portSymbol.getName() + ";")
                        .sorted()
                        .collect(Collectors.joining("\n"));
            }
        } else {
            this.instruction = NameHelper.getAllFieldNames(rosMsg).stream()
                    .map(field -> "tmpMsg." + field + " = component->" + portSymbol.getName() + "." + field + ";")
                    .sorted()
                    .collect(Collectors.joining("\n"));
        }
    }
}
