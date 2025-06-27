/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.loopSolver.odeint;

import de.monticore.lang.monticar.generator.cmake.CMakeFindModule;
import org.apache.commons.lang3.SystemUtils;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;

public class OdeintOptions {

    private static Collection<CMakeFindModule> dependencies = null;

    public static Collection<CMakeFindModule> getDependencies() {
        if (dependencies == null) {
            dependencies = new ArrayList<>();

            CMakeFindModule BOOST = new CMakeFindModule("BOOST",
                    "boost/version.hpp", "",
                    new ArrayList(), new ArrayList(), new ArrayList<>(), new ArrayList(), new ArrayList(),
                    true, false, true);
            dependencies.add(BOOST);
        }
        return dependencies;
    }
}
