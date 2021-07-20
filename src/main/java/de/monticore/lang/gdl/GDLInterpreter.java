package de.monticore.lang.gdl;

import java.io.IOException;
import java.math.BigInteger;
import java.util.ArrayList;
import java.util.List;
import java.util.Map; 
import java.util.Optional;
import java.util.Scanner;
import java.util.TreeMap;
import java.util.stream.Collectors;

import org.antlr.v4.runtime.RecognitionException;
import org.apache.commons.collections4.BidiMap;
import org.apache.commons.collections4.bidimap.DualTreeBidiMap;
// import org.sosy_lab.common.ShutdownManager;
import org.sosy_lab.common.configuration.Configuration;
import org.sosy_lab.common.configuration.InvalidConfigurationException;
import org.sosy_lab.common.log.BasicLogManager;
import org.sosy_lab.common.log.LogManager;
import org.sosy_lab.java_smt.SolverContextFactory;
import org.sosy_lab.java_smt.SolverContextFactory.Solvers;
import org.sosy_lab.java_smt.api.BooleanFormula;
import org.sosy_lab.java_smt.api.BooleanFormulaManager;
import org.sosy_lab.java_smt.api.IntegerFormulaManager;
import org.sosy_lab.java_smt.api.Model;
import org.sosy_lab.java_smt.api.ProverEnvironment;
import org.sosy_lab.java_smt.api.SolverContext;
import org.sosy_lab.java_smt.api.SolverException;
import org.sosy_lab.java_smt.api.Model.ValueAssignment;
import org.sosy_lab.java_smt.api.NumeralFormula.IntegerFormula;
import org.sosy_lab.java_smt.api.SolverContext.ProverOptions;

import de.monticore.lang.gdl._ast.ASTGame;
import de.monticore.lang.gdl._ast.ASTGameExpression;
import de.monticore.lang.gdl._ast.ASTGameFunction;
import de.monticore.lang.gdl._ast.ASTGameInit;
import de.monticore.lang.gdl._ast.ASTGameValue;
import de.monticore.lang.gdl._parser.GDLParser;
import de.monticore.lang.gdl._symboltable.GDLScopesGenitor;
import de.monticore.lang.gdl._symboltable.IGDLArtifactScope;
import de.monticore.lang.gdl._symboltable.IGDLGlobalScope;
import de.monticore.lang.gdl._visitor.GDLTraverser;
import de.se_rwth.commons.logging.Log;

public class GDLInterpreter {

    public static void main(String[] args) throws IOException, InvalidConfigurationException, InterruptedException, SolverException {
        if (args.length != 1) {
            Log.error("Specify exactly one model file.");
            return;
        }

        // Parse model to ast
        String modelFileName = args[0];
        final ASTGame ast = parse(modelFileName);
        final IGDLArtifactScope scope = createSymbolTable(ast);

        final Interpreter interpreter = new Interpreter(ast, scope).init();
        
        // System.out.println(scope.getGameFunctionDefinitionSymbols());
        // System.out.println(scope.getGameFunctionDefinitionSymbols().get("isLegalMove").get(0).getAstNode());

        // Initialize SAT Solver objects
        

        // Generate prover environment
        // ProverEnvironment prover = context.newProverEnvironment(ProverOptions.GENERATE_MODELS);
        // prover.addConstraint(volatileState.getConstraint());
        // prover.addConstraint(constantState.getConstraint());


        // boolean isUnsat = prover.isUnsat();
        // if (isUnsat) {
        //     System.out.println("UNSAT");
        // } else {
        //     // generate model
        //     Model model = prover.getModel();

        //     // sort value assignments by name
        //     List<ValueAssignment> valueAssignments = model.asList()
        //         .stream()
        //         .sorted((a, b) -> a.getName().compareTo(b.getName()))
        //         .collect(Collectors.toList());

        //     // print all value assignments
        //     for (ValueAssignment va : valueAssignments) {
        //         String argName = va.getName();
        //         // Reverse lookup int assignment for value name
        //         String argValue = constValueMap.getKey(((BigInteger) va.getValue()).intValue());
        //         Log.println(String.format("%s = %s", argName, argValue));
        //     }
        // }
        
        // REMOVE LATER
        interpreter.interpret("");


        Scanner s = new Scanner(System.in);
        String line;
        while (!(line = s.nextLine()).equals("/exit") && line != null) {
            interpreter.interpret(line);
        }
        s.close();
    }

    /**
     * Parse the model contained in the specified file.
     *
     * @param model - file to parse
     * @return AST of the model.
     */
    public static ASTGame parse(String model) {
        try {
            GDLParser parser = GDLMill.parser();
            Optional<ASTGame> optGame = parser.parse(model);

            if (!parser.hasErrors() && optGame.isPresent()) {
                return optGame.get();
            }
            Log.error("Model could not be parsed.");
        } catch (RecognitionException | IOException e) {
            Log.error("Failed to parse " + model, e);
        }
        return null;
    }

    /**
     * Create the symbol table from the parsed AST.
     *
     * @param ast Input AST.
     * @return The symbol table created from the AST.
     */
    public static IGDLArtifactScope createSymbolTable(ASTGame ast) {
        IGDLGlobalScope gs = GDLMill.globalScope();
        gs.clear();

        GDLScopesGenitor genitor = GDLMill.scopesGenitor();
        GDLTraverser traverser = GDLMill.traverser();
        traverser.setGDLHandler(genitor);
        traverser.add4GDL(genitor);
        genitor.putOnStack(gs);

        IGDLArtifactScope scope = genitor.createFromAST(ast);
        gs.addSubScope(scope);
        return scope;
    }
    
    // public static void example() throws InvalidConfigurationException, InterruptedException, SolverException {
    //     Configuration config = Configuration.defaultConfiguration();
    //     LogManager logger = BasicLogManager.create(config);
    //     ShutdownManager shutdown = ShutdownManager.create();

    //     SolverContext context = SolverContextFactory.createSolverContext(config, logger, shutdown.getNotifier(),
    //             Solvers.SMTINTERPOL);

    //     ArrayFormulaManager amgr = context.getFormulaManager().getArrayFormulaManager();
    //     IntegerFormulaManager imgr = context.getFormulaManager().getIntegerFormulaManager();
    //     BooleanFormulaManager bmgr = context.getFormulaManager().getBooleanFormulaManager();

    //     IntegerFormula x = imgr.makeVariable("x");
    //     IntegerFormula y = imgr.makeVariable("y");
    //     IntegerFormula one = imgr.makeNumber(1);
    //     IntegerFormula two = imgr.makeNumber(2);

    //     BooleanFormula f = imgr.equal(x, one);
    //     BooleanFormula g = imgr.equal(y, two);
    //     BooleanFormula prop = bmgr.and(f, g);

    //     ProverEnvironment prover = context.newProverEnvironment(ProverOptions.GENERATE_MODELS);
    //     prover.addConstraint(imgr.equal(x, y));

    //     boolean isUnsat = prover.isUnsat();
    //     if (isUnsat) {
    //         System.out.println("UNSAT");
    //     } else {
    //         Model model = prover.getModel();
    //         System.out.printf("SAT with x = %s, y = %s\n", model.evaluate(x), model.evaluate(y));
    //         System.out.println(model.evaluate(prop));

    //         for (ValueAssignment va : prover.getModelAssignments()) {
    //             System.out.println(va);
    //         }

    //     }
    // }

}
