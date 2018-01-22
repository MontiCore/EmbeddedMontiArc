package de.monticore.lang.monticar.generator.roscpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;

public class MethodHelper {

    //TODO: refactor
    public static String getPortNameTargetLanguage(PortSymbol portSymbol) {
        //TODO: arrays
        //TODO: check format
        //TODO: from cpp generator?
        return portSymbol.getName();
    }
}
