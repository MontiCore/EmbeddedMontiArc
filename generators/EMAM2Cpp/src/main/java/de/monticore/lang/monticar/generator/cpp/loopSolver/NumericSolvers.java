/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.loopSolver;

import de.monticore.lang.monticar.generator.cpp.loopSolver.daecpp.DAECPPSolveGenerator;
import de.monticore.lang.monticar.generator.cpp.loopSolver.odeint.ODEIntSolveGenerator;

public class NumericSolvers {

    public static NonlinearSolveGenerator getNonLinearSolveGenerator() {
        return new DAECPPSolveGenerator();
    }

    public static ODESolveGenerator getODESolveGenerator() {
        return new ODEIntSolveGenerator();
    }

    public static DAESolveGenerator getDAESolveGenerator() {
        return new DAECPPSolveGenerator();
    }

}
