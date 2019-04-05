package de.monticore.lang.monticar.generator.roscpp.instructions;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.monticar.common2._ast.ASTCommonMatrixType;
import de.monticore.lang.monticar.generator.roscpp.helper.IndexHelper;
import de.monticore.lang.monticar.generator.roscpp.helper.NameHelper;
import de.monticore.lang.monticar.generator.rosmsg.RosMsg;
import de.monticore.lang.monticar.struct._symboltable.StructSymbol;
import de.monticore.lang.monticar.ts.MCASTTypeSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

public class SetStructPortInstruction{
    private SetStructPortInstruction() {
    }

    public static String getInstruction(EMAPortSymbol port, RosMsg rosMsg) {
        String inst;
        if (rosMsg.getName().startsWith("std_msgs/")) {
            if (rosMsg.getName().endsWith("MultiArray")) {
                ASTCommonMatrixType matrixType = (ASTCommonMatrixType) ((MCASTTypeSymbol) port.getTypeReference().getReferencedSymbol()).getAstType();
                List<String> dimSizes = IndexHelper.getDimSizesOfMatrixType(matrixType);

                inst = "";
                inst += "int counter = 0;\n";
                String indexString = "";
                for (int i = 0; i < dimSizes.size(); i++) {
                    String curInd = "i" + i;
                    indexString += (i == 0 ? "" : ", ") + curInd;
                    inst += "for(int " + curInd + " = 0; " + curInd + " < " + dimSizes.get(i) + "; " + curInd + "++){\n";
                }

                inst += "(component->" + NameHelper.getPortNameTargetLanguage(port) + ")(" + indexString + ") = msg->data[counter]";
                //TODO: check type not name
                if (rosMsg.getName().equals("std_msgs/ByteMultiArray")) {
                    //is a bool msg
                    inst += " != 0";
                }
                inst += ";\n";
                inst += "counter++;\n";

                for (int i = 0; i < dimSizes.size(); i++) {
                    inst += "}\n";
                }


            } else {
                inst = NameHelper.getAllFieldNames(rosMsg).stream()
                        .map(field -> "component->" + NameHelper.getPortNameTargetLanguage(port) + " = msg->" + field + ";")
                        .sorted()
                        .collect(Collectors.joining("\n"));
            }
        } else {
            StructSymbol structSymbol = (StructSymbol) port.getTypeReference().getReferencedSymbol();

            List<String> structFieldNames = NameHelper.getAllFieldNames(structSymbol);
            List<String> rosMsgFieldNames = NameHelper.getAllFieldNames(rosMsg);

            if(structFieldNames.size() != rosMsgFieldNames.size()){
                Log.error("Struct and RosMsg don't have the same number of fields!");
            }

            // Struct field names can contain uppercase letters
            // RosMsgs for ros2 can only contain lowercase letters
            // => match each sField to exactly one rField
            Map<String, String> structToMsgField = new HashMap<>();
            for (String sField : structFieldNames){
                boolean found = false;
                for(String rField : rosMsgFieldNames){
                    if(sField.equals(rField) || sField.toLowerCase().equals(rField)){
                        if(found){
                            Log.error("Found more than one RosMsg field for one Struct field!");
                        }else{
                            structToMsgField.put(sField, rField);
                            found = true;
                        }
                    }
                }
                if(!found){
                    Log.error("No RosMsg field found for Struct field " + sField);
                }
            }

            inst = structFieldNames.stream()
                    .map(field -> "component->" + NameHelper.getPortNameTargetLanguage(port) + "." + field + " = msg->" + structToMsgField.get(field) + ";")
                    .sorted()
                    .collect(Collectors.joining("\n"));
        }
        return inst;
    }

}
