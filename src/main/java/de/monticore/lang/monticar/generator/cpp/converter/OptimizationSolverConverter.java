/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.converter;

import de.monticore.lang.math._symboltable.expression.MathValueSymbol;
import de.monticore.lang.math._symboltable.expression.MathValueType;
import de.monticore.lang.mathopt._symboltable.MathOptimizationStatementSymbol;
import de.monticore.lang.monticar.generator.BluePrint;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.cpp.BluePrintCPP;
import de.monticore.lang.monticar.generator.cpp.GeneratorEMAMOpt2CPP;
import de.monticore.lang.monticar.generator.cpp.optimizationSolver.problem.OptimizationProblemClassification;
import de.monticore.lang.monticar.generator.cpp.optimizationSolver.problem.Problem;
import de.monticore.lang.monticar.generator.cpp.optimizationSolver.solver.SolverGenerator;
import de.monticore.lang.monticar.generator.cpp.optimizationSolver.solver.factory.SolverGeneratorFactory;
import de.monticore.lang.monticar.types2._ast.ASTElementType;
import de.se_rwth.commons.logging.Log;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * Manages the conversion from a MathOptimizationSymbol to executable C++ code.
 *
 */
public class OptimizationSolverConverter {

    // singleton implementation
    private static OptimizationSolverConverter ourInstance = new OptimizationSolverConverter();

    public static OptimizationSolverConverter getInstance() {
        return ourInstance;
    }

    private OptimizationSolverConverter() {
    }

    // methods

    /**
     * Generates code solving command including auxiliary code files.
     *
     * @param symbol         MathExpressionSymbol from which code should be generated
     * @param includeStrings Additional include strings which are necessary to execute the code
     * @return Code line which will execute the optimization solver on the optimization problem
     */
    public static String getOptimizationExpressionCode(MathOptimizationStatementSymbol symbol, List<String> includeStrings, BluePrintCPP bluePrint) {

        // first step: decide for correct solver for problem class
        OptimizationProblemClassification problemClassification = new OptimizationProblemClassification(symbol);
        Problem problemType = problemClassification.getProblemType();

        // second step: decide for implementation
        GeneratorEMAMOpt2CPP gen = getInstance().getGenerator(bluePrint);
        SolverGenerator solverGenerator = SolverGeneratorFactory.getInstance().createPreferredSolverForProblem(problemType, gen.getPreferedSolver(), gen.forceUsePreferredSolver());

        // third step: generate code from solver generator

        // declare needed variables
        String result = getOutputVariableDeclarations(symbol, problemType, bluePrint);

        // call solverGenerator
        ArrayList<FileContent> auxiliaryFiles = new ArrayList<FileContent>();
        result += solverGenerator.generateSolverInstruction(problemType, auxiliaryFiles, bluePrint);

        // also generate auxiliaryFiles
        try {
            getInstance().getGenerator(bluePrint).saveFilesToDisk(auxiliaryFiles);
        } catch (IOException e) {
            Log.error(e.toString());
        }

        // do not forget to add include strings
        bluePrint.additionalIncludeStrings.addAll(solverGenerator.getNecessaryIncludes());

        return result;
    }


    private static boolean isOptimizationVariableAlreadyDefined(MathValueSymbol optimizationVariable, BluePrintCPP bluePrint) {
        boolean defined = false;
        if (optimizationVariable.getType() == null) {
            defined = true;
        } else if (bluePrint.getMathInformationRegister().getMathValueSymbol(optimizationVariable.getName()) != null) {
            defined = true;
        }
        return defined;
    }

    private static String getOutputVariableDeclarations(MathOptimizationStatementSymbol symbol, Problem problemType, BluePrintCPP bluePrint) {
        String result = "";
        if (!isOptimizationVariableAlreadyDefined(symbol.getOptimizationVariable(), bluePrint))
            result += ExecuteMethodGenerator.generateExecuteCode(symbol.getOptimizationVariable(), new ArrayList<>());
        if (symbol.hasReturnValue()) {
            MathValueSymbol expr = symbol.getObjectiveValue();
            MathValueSymbol decl = new MathValueSymbol(expr.getName());
            decl.setType(expr.getType());
            result += ExecuteMethodGenerator.generateExecuteCode(decl, new ArrayList<>());
        } else {
            problemType.setObjectiveValueVariable("objectiveValue" + problemType.getId());
            MathValueSymbol decl = new MathValueSymbol(problemType.getObjectiveValueVariable());
            MathValueType type = new MathValueType();
            ASTElementType astType = new ASTElementType();
            astType.setName("Q");
            type.setType(astType);
            decl.setType(type);
            result += ExecuteMethodGenerator.generateExecuteCode(decl, new ArrayList<>());
        }
        return result;
    }

    protected GeneratorEMAMOpt2CPP getGenerator(BluePrint bluePrint) {
        GeneratorEMAMOpt2CPP g = null;
        if (bluePrint.getGenerator() instanceof GeneratorEMAMOpt2CPP)
            g = (GeneratorEMAMOpt2CPP) bluePrint.getGenerator();
        return g;
    }

}
