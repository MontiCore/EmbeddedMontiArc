/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.roscpp.helper;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.ros.RosConnectionSymbol;
import de.monticore.lang.monticar.generator.rosmsg.RosMsg;
import de.monticore.lang.monticar.generator.rosmsg.util.FileContent;
import de.monticore.lang.monticar.struct._symboltable.StructSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

public class NameHelper {

    private NameHelper() {
    }

    public static String getPortNameTargetLanguage(EMAPortSymbol portSymbol) {
        //TODO: get from cpp generator for consistency?
        if (portSymbol.isPartOfPortArray()) {
            return getFixedArrayPortName(portSymbol);
        } else {
            return portSymbol.getName();
        }
    }

    public static String getFixedArrayPortName(EMAPortSymbol portSymbol) {
        if (!portSymbol.isPartOfPortArray())
            throw new IllegalArgumentException("EMAPortSymbol " + portSymbol.getName() + " is not part of an array!");

        return fixName(portSymbol.getName());
    }

    private static String fixName(String name) {
        String shortName = name.substring(0, name.lastIndexOf('['));
        String indexString = name.substring(name.lastIndexOf('['), name.lastIndexOf(']') + 1);
        String rest = name.substring(name.lastIndexOf("]") + 1, name.length());

        int emamIndex = Integer.parseInt(indexString.replace("[", "").replace("]", ""));
        if (emamIndex == 0) {
            throw new IllegalArgumentException("The index of " + name + " is 0 but EMAM indices are 1 based!");
        }

        int cppIndex = emamIndex - 1;

        return shortName + "[" + cppIndex + "]" + rest;
    }

    public static String getAdapterName(EMAComponentInstanceSymbol componentInstanceSymbol) {
        return "RosAdapter_" + getComponentNameTargetLanguage(componentInstanceSymbol.getFullName());
    }

    public static String getComponentNameTargetLanguage(String fullName) {
        return fullName
                .replace('.', '_')
                .replace('[', '_')
                .replace(']', '_');
    }

    public static String getPackageOfMsgType(String msgType) {
        if (!msgType.contains("/")) {
            Log.error("Msg types must contain package name! " + msgType + " doesn't!");
            return null;
        } else {
            return msgType.split("/")[0];
        }

    }

    public static List<String> getAllFieldNames(RosMsg rosMsg) {
        ArrayList<String> res = new ArrayList<>();
        rosMsg.getFields().forEach(f -> {
            if (f.getType() instanceof RosMsg) {
                List<String> tmpFields = getAllFieldNames(((RosMsg) f.getType()));
                tmpFields.stream()
                        .map(tmpF -> f.getName() + "." + tmpF)
                        .forEach(res::add);
            } else {
                res.add(f.getName());
            }
        });
        return res;
    }

    public static List<String> getAllFieldNames(StructSymbol structSymbol){
        ArrayList<String> res = new ArrayList<>();
        structSymbol.getStructFieldDefinitions().forEach(f -> {
            if (f.getType().getReferencedSymbol() instanceof StructSymbol) {
                List<String> tmpFields = getAllFieldNames((StructSymbol) f.getType().getReferencedSymbol());
                tmpFields.stream()
                        .map(tmpF -> f.getName() + "." + tmpF)
                        .forEach(res::add);
            } else {
                res.add(f.getName());
            }
        });
        return res;
    }

    public static String addMsgToMsgType(String msgType){
        String[] parts = msgType.split("/");
        if (!parts[1].equals("msg")) {
            parts[0] = parts[0] + "/msg";
        }
        return String.join("/", parts);

    }

    public static String msgTypeToSnakecase(String type) {
        String[] parts = type.split("/");
        // First letter lowercase without _
        parts[parts.length - 1] = parts[parts.length - 1].substring(0, 1).toLowerCase() + parts[parts.length - 1].substring(1);
        // After: replace A with _a
        parts[parts.length - 1] = parts[parts.length - 1].replaceAll("([A-Z])", "_$1").toLowerCase();
        return String.join("/", parts);
    }

    public static String getTopicNameTargetLanguage(String topicName) {
        return topicName.replace("/", "_")
                .replace("[", "_")
                .replace("]", "_");
    }

    public static String getFullRosType(RosConnectionSymbol rosConnectionSymbol) {
        return rosConnectionSymbol.getTopicType().get().replace("/", "::");
    }
}
