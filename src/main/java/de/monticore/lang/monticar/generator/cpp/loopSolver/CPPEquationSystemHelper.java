/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.loopSolver;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.monticar.generator.cmake.CMakeFindModule;
import de.monticore.lang.monticar.semantics.helper.NameHelper;
import org.apache.commons.lang3.SystemUtils;

import java.util.*;

public class CPPEquationSystemHelper {

    public static String getNameOfPortOfComponent(EMAPortInstanceSymbol port) {
        String componentName = getNameOfComponent(port.getComponentInstance());
        return String.join(".", componentName, port.getName());
    }

    public static String getNameOfPort(EMAPortInstanceSymbol port) {
        return NameHelper.replaceWithUnderScore(NameHelper.calculateFullQualifiedNameOf(port));
    }

    public static String getNameOfComponent(EMAComponentInstanceSymbol component) {
        return NameHelper.replaceWithUnderScore(NameHelper.calculateFullQualifiedNameOf(component));
    }

}
