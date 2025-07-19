/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.loopSolver.daecpp;

import de.monticore.lang.monticar.generator.cmake.CMakeFindModule;
import org.apache.commons.lang3.SystemUtils;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;

public class DAECPPOptions {

    private static Collection<CMakeFindModule> dependencies = null;

    public static Collection<CMakeFindModule> getDependencies() {
        if (dependencies == null) {
            boolean is64bit = false;
            if (System.getProperty("os.name").contains("Windows")) {
                is64bit = (System.getenv("ProgramFiles(x86)") != null);
            } else {
                is64bit = (System.getProperty("os.arch").indexOf("64") != -1);
            }

            dependencies = new ArrayList<>();

            CMakeFindModule DAECPP = new CMakeFindModule("DAECPP",
                    "solver.h", "daecpp_static",
                    new ArrayList(), new ArrayList(), new ArrayList(), new ArrayList(), new ArrayList(),
                    true, true, true);

            List<String> mkllibSuffix = Arrays.asList("lib/intel64_win", "lib/ia32_win", "lib/intel64");
            List<String> intellibSuffix = Arrays.asList("../compiler/lib/intel64_win", "../compiler/lib/ia32_win", "../lib/intel64", "../lib");

            CMakeFindModule MKL_INCLUDE = new CMakeFindModule("MKL",
                    "mkl.h", "",
                    new ArrayList(), new ArrayList(), new ArrayList(), new ArrayList(), new ArrayList(),
                    true, false, true);

            CMakeFindModule LIB_MKL_INTEL_LP64 = new CMakeFindModule("MKL_INTEL_LP64",
                    "", "mkl_intel_lp64 mkl_intel_c mkl_intel_ilp64 mkl_intel_lp64",
                    new ArrayList(),
                    new ArrayList(),
                    new ArrayList(),
                    mkllibSuffix,
                    Arrays.asList("MKL_HOME"),
                    false, true, true);

            CMakeFindModule LIB_MKL_THREAD = new CMakeFindModule("MKL_THREAD",
                    "", "mkl_intel_thread",
                    new ArrayList(),
                    new ArrayList(),
                    new ArrayList(),
                    mkllibSuffix,
                    Arrays.asList("MKL_HOME"),
                    false, true, true);

            CMakeFindModule LIB_MKL_CORE = new CMakeFindModule("MKL_CORE",
                    "", "mkl_core",
                    new ArrayList(),
                    new ArrayList(),
                    new ArrayList(),
                    mkllibSuffix,
                    Arrays.asList("MKL_HOME"),
                    false, true, true);

            CMakeFindModule LIB_INTEL_OPENMP = new CMakeFindModule("INTEL_OPENMP",
                    "", "libiomp5md iomp5",
                    new ArrayList(),
                    new ArrayList(),
                    new ArrayList(),
                    intellibSuffix,
                    Arrays.asList("MKL_HOME"),
                    false, true, true);

            dependencies.addAll(Arrays.asList(DAECPP, MKL_INCLUDE, LIB_MKL_INTEL_LP64, LIB_MKL_THREAD, LIB_MKL_CORE, LIB_INTEL_OPENMP));

            if (!SystemUtils.IS_OS_WINDOWS) {
                CMakeFindModule LIB_PTHREAD = new CMakeFindModule("PTHREAD",
                        "", "pthread",
                        new ArrayList(), new ArrayList(), new ArrayList(), new ArrayList(), new ArrayList(),
                        false, true, false);

                CMakeFindModule LIB_MATH = new CMakeFindModule("MATH",
                        "", "m",
                        new ArrayList(), new ArrayList(), new ArrayList(), new ArrayList(), new ArrayList(),
                        false, true, false);

                CMakeFindModule LIB_DL = new CMakeFindModule("DL",
                        "", "dl",
                        new ArrayList(), new ArrayList(), new ArrayList(), new ArrayList(), new ArrayList(),
                        false, true, false);
                dependencies.addAll(Arrays.asList(LIB_PTHREAD, LIB_MATH, LIB_DL));
            }
        }

        return dependencies;
    }
}
