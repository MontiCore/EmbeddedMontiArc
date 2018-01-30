package de.monticore.lang.monticar.generator.roscpp.helper;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;

public class PortNameHelper {

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
}
