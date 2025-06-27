/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.viewmodel;

import de.monticore.symboltable.Symbol;

public final class Utils {

    private Utils() {
    }

    public static String getIncludeName(Symbol s) {
        return s.getFullName().replace('.', '_');
    }
}
