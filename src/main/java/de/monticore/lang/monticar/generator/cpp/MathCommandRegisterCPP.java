package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.monticar.generator.MathCommandRegister;
import de.monticore.lang.monticar.generator.cpp.commands.*;

/**
 * @author Sascha Schneiders
 */
public class MathCommandRegisterCPP extends MathCommandRegister {

    @Override
    protected void init() {

        //registerMathCommand("size", "testo");
        registerMathCommand(new MathAtan2Command());
        registerMathCommand(new MathLog2Command());
        registerMathCommand(new MathSizeCommand());
        registerMathCommand(new MathSumCommand());
        registerMathCommand(new MathEyeCommand());
        registerMathCommand(new MathNormCommand());
        registerMathCommand(new MathDiagCommand());
        registerMathCommand(new MathEigvalCommand());
        registerMathCommand(new MathEigvecCommand());
        registerMathCommand(new MathGcdCommand());
        registerMathCommand(new MathInvCommand());
        registerMathCommand(new MathInvDiagCommand());
        registerMathCommand(new MathAbsCommand());
        registerMathCommand(new MathAcosCommand());
        registerMathCommand(new MathAcoshCommand());
        registerMathCommand(new MathAsinCommand());
        registerMathCommand(new MathAsinhCommand());
        registerMathCommand(new MathAtanCommand());
        registerMathCommand(new MathAtanhCommand());
        registerMathCommand(new MathCosCommand());
        registerMathCommand(new MathCoshCommand());
        registerMathCommand(new MathExpCommand());
        registerMathCommand(new MathLogCommand());
        registerMathCommand(new MathLog10Command());
        registerMathCommand(new MathSinCommand());
        registerMathCommand(new MathSinhCommand());
        registerMathCommand(new MathSqrtCommand());
        registerMathCommand(new MathTanCommand());
        registerMathCommand(new MathTanhCommand());
        registerMathCommand(new MathMinCommand());
        registerMathCommand(new MathMaxCommand());
        registerMathCommand(new MathOnesCommand());
        registerMathCommand(new MathZerosCommand());
        registerMathCommand(new MathDetCommand());
        registerMathCommand(new MathKMeansCommand());
        registerMathCommand(new MathSqrtmCommand());
        registerMathCommand(new MathSqrtmDiagCommand());
    }
}
