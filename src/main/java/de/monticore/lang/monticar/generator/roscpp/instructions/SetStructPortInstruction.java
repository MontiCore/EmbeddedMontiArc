/* (c) https://github.com/MontiCore/monticore */
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

    public static String getStructInstruction(EMAPortSymbol port, RosMsg rosMsg, String fieldPrefix) {
        String inst;
        if (rosMsg.getName().startsWith("std_msgs/")) { 
                inst = NameHelper.getAllFieldNames(rosMsg).stream()
                        .map(field -> "component->" + NameHelper.getPortNameTargetLanguage(port) + " = msg->" + fieldPrefix + field + ";")
                        .sorted()
                        .collect(Collectors.joining("\n"));
        } else { //port is a struct -> further handling necessary
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
                    .map(field -> "component->" + NameHelper.getPortNameTargetLanguage(port) + "." + field + " = msg->" + fieldPrefix + structToMsgField.get(field) + ";")
                    .sorted()
                    .collect(Collectors.joining("\n"));
        }
        return inst;
    }

    public static String getMatrixInstruction(EMAPortSymbol port, RosMsg rosMsg, String fieldPrefix){
                    String inst;
                ASTCommonMatrixType matrixType = (ASTCommonMatrixType) ((MCASTTypeSymbol) port.getTypeReference().getReferencedSymbol()).getAstType();
                List<String> dimSizes =  IndexHelper.getDimSizesOfMatrixType(matrixType);	

                inst = "";
                inst += "int counter = 0;\n";
                String indexString = "";
                for (int i = 0; i < dimSizes.size(); i++) {
                    String curInd = "i" + i;
                    indexString += (i == 0 ? "" : ", ") + curInd;
                    inst += "for(int " + curInd + " = 0; " + curInd + " < " + dimSizes.get(i) + "; " + curInd + "++){\n";
                }

		    
                    //part of the improved array handling
                    String upperBound;
                    String lowerBound;

                    if (!fieldPrefix.isEmpty() && fieldPrefix.contains(":")){ //syntax correct?
                        String split[] = fieldPrefix.split(":", 2);
                        boolean boundExists = !split[0].replaceAll("[^0-9]", "").isEmpty() && !split[0].replaceAll("[^0-9]", "").equals("0"); //0 is not a valid bound in EMAM

                        //extract bounds from msgField
                        lowerBound = boundExists ? split[0].replaceAll("[^0-9]", "")+"-1" : "0";
                        boundExists = !split[1].replaceAll("[^0-9]", "").isEmpty() && !split[1].replaceAll("[^0-9]", "").equals("0"); //0 is not a valid bound in EMAM;
                        upperBound = boundExists ? split[1].replaceAll("[^0-9]", "")+"-1" : "msg->data.size()-1";  
                    } else { //no bounds given
                        lowerBound = "0";
                        upperBound = "msg->data.size()-1";
                    }


		    if(!upperBound.equals("")){
//		        if (Integer.parseInt(lowerBound) > Integer.parseInt(upperBound)) { If 
//		            Log.error(" ArrayBoundsHandler: lowerBound > upperBound!");
//		        }
		        inst += "if(" + lowerBound + " <= counter && counter <= " + upperBound + "){\n";
                    }
 
                    //three if-cases will be generated: lB <= counter <= uB, counter < lB, counter > uB
                    String tmp;
                    if (fieldPrefix.isEmpty()){
                        tmp = "(component->" + NameHelper.getPortNameTargetLanguage(port) + ")(" + indexString + ") = msg->" + fieldPrefix + "data[counter]";
                    } else {
                        tmp = "(component->" + NameHelper.getPortNameTargetLanguage(port) + ")(" + indexString + "-" + lowerBound + ") = msg->" + fieldPrefix.replaceAll("[0-9]", "").replace(":","counter");
                        tmp = tmp.substring(0,tmp.length()-1);
                    }
                    inst += tmp;
	                  //TODO: check type not name
	                  if (rosMsg.getName().equals("std_msgs/ByteMultiArray")) {
			                //is a bool msg
			                inst += " != 0";
			              }
	 
		                inst += ";\n";
                    if(!upperBound.equals("")){
		                  inst += "}\n";
		                  
		                  inst += "else if(" + lowerBound + " > counter){\n";
		                  inst += "(component->" + NameHelper.getPortNameTargetLanguage(port) + ")(" + indexString + "+" + upperBound + "-" + lowerBound + "+1" +") = 0" + ";\n";
		                  inst += "}\n";

		                  inst += "else{\n";
		                  inst += "(component->" + NameHelper.getPortNameTargetLanguage(port) + ")(" + indexString + ") = 0" + ";\n";
		                  inst += "}\n";
                    }
                
                    inst += "counter++;\n";

                    for (int i = 0; i < dimSizes.size(); i++) {
                        inst += "}\n";
                    }
                    return inst;       
    }
}
