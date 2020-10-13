/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.cpp.converter.ExecuteMethodGenerator;
import de.monticore.lang.monticar.generator.cpp.converter.OptimizationSymbolHandler;
import de.monticore.lang.monticar.generator.cpp.optimizationSolver.solver.Solver;
import de.monticore.lang.monticar.generator.cpp.optimizationSolver.solver.SolverOptions;
import de.monticore.lang.tagging._symboltable.TaggingResolver;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * Generates C++ code out of EmbeddedMontiArcMathOpt (EMAMOpt) models Models.
 *
 */
public class GeneratorEMAMOpt2CPP extends GeneratorCPP{

    // fields
    private OptimizationSymbolHandler executeMethodGeneratorOpt = new OptimizationSymbolHandler();
    private MathOptFunctionFixer mathOptFunctionFixer = new MathOptFunctionFixer();
    private Solver preferedSolver = Solver.none;
    private boolean forceUsePreferredSolver = false;
    private SolverOptions solverOptions = new SolverOptions();

    public GeneratorEMAMOpt2CPP() {
        super();
        setup();
    }

    protected void setup() {
        setGenerateCMake(true);
        // always use armadillo by default
        useArmadilloBackend();
        // extend ExecuteMethodGenerator
        executeMethodGeneratorOpt.setSuccessor(ExecuteMethodGenerator.getInstance());
        mathOptFunctionFixer.setSuccessor(MathFunctionFixer.getInstance());
    }

    public List<File> generate(EMAComponentInstanceSymbol componentInstanceSymbol, TaggingResolver taggingResolver) throws IOException {
        List<File> files = new ArrayList<>();
        files.addAll(generateFiles(componentInstanceSymbol, taggingResolver));
        if (isGenerateCMakeEnabled())
            files.addAll(generateCMakeFiles(componentInstanceSymbol));
        return files;
    }

    public boolean willAccept(EMAComponentInstanceSymbol componentInstanceSymbol) {
        return true;
    }

    public OptimizationSymbolHandler getExecuteMethodGeneratorOpt() {
        return executeMethodGeneratorOpt;
    }

    public MathOptFunctionFixer getMathOptFunctionFixer() {
        return mathOptFunctionFixer;
    }

    public Solver getPreferedSolver() {
        return preferedSolver;
    }

    public void setPreferedSolver(Solver preferedSolver) {
        this.preferedSolver = preferedSolver;
    }

    public SolverOptions getSolverOptions() {
        return solverOptions;
    }

    public void setSolverOptions(SolverOptions solverOptions) {
        this.solverOptions = solverOptions;
    }

    public boolean forceUsePreferredSolver() {
        return forceUsePreferredSolver;
    }

    public void setForceUsePreferredSolver(boolean forceUsePreferredSolver) {
        this.forceUsePreferredSolver = forceUsePreferredSolver;
    }
}
