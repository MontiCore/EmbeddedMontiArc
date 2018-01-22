package de.monticore.lang.monticar.generator.roscpp;

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

        String name = portSymbol.getName();
        String shortName = name.substring(0, name.lastIndexOf('['));
        String indexString = name.substring(name.lastIndexOf('['), name.lastIndexOf(']')).replace("[", "");

        int emamIndex = Integer.parseInt(indexString);
        int cppIndex = emamIndex - 1;

        return shortName + "[" + cppIndex + "]";
    }
}
