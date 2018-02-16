package de.monticore.lang.monticar.generator.roscpp.helper;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.monticar.generator.rosmsg.RosMsg;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;

public class NameHelper {

    private NameHelper() {
    }

    public static String getPortNameTargetLanguage(PortSymbol portSymbol) {
        //TODO: get from cpp generator for consistency?
        if (portSymbol.isPartOfPortArray()) {
            return getFixedArrayPortName(portSymbol);
        } else {
            return portSymbol.getName();
        }
    }

    public static String getFixedArrayPortName(PortSymbol portSymbol) {
        if (!portSymbol.isPartOfPortArray())
            throw new IllegalArgumentException("PortSymbol " + portSymbol.getName() + " is not part of an array!");

        return fixName(portSymbol.getName());
    }

    public static String getFixedMsgFieldName(String name) {
        if (!(name.contains("[") && name.contains("]"))) {
            throw new IllegalArgumentException("The MsgField " + name + "is not part of an array!");
        }

        return fixName(name);
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

    public static String getAdapterName(ExpandedComponentInstanceSymbol componentInstanceSymbol) {
        return "RosAdapter_" + componentInstanceSymbol.getFullName().replace('.', '_');
    }

    public static String getComponentNameTargetLanguage(String fullName) {
        return fullName.replace(".", "_");
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
}
