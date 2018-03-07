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

public class SetStructMsgInstruction extends TargetCodeInstruction {
    public SetStructMsgInstruction(PortSymbol portSymbol, RosMsg rosMsg) {
        if (rosMsg.getName().startsWith("std_msgs/")) {
            if (rosMsg.getName().endsWith("MultiArray")) {
                ASTCommonMatrixType matrixType = (ASTCommonMatrixType) ((MCASTTypeSymbol) portSymbol.getTypeReference().getReferencedSymbol()).getAstType();
                List<Long> strides = IndexHelper.getStridesOfDimSizes(IndexHelper.getDimSizesOfMatrixType(matrixType));
                List<String> indexStrings = IndexHelper.getIndexStrings(strides);
                int i = 0;
                this.instruction = "tmpMsg.data.resize(" + strides.get(0) + ");\n";
                for (String indexString : indexStrings) {
                    this.instruction += "tmpMsg.data[" + i + "] = (component->" + NameHelper.getPortNameTargetLanguage(portSymbol) + ")(" + indexString + ")";
                    //TODO: check type not name
                    if (rosMsg.getName().equals("std_msgs/ByteMultiArray")) {
                        //is a bool msg
                        this.instruction += " ? 1 : 0";
                    }
                    this.instruction += ";\n";
                    i++;
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
