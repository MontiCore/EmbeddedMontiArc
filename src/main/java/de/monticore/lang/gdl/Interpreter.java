package de.monticore.lang.gdl;

import java.io.IOException;
import java.math.BigInteger;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import org.apache.commons.collections4.BidiMap;
import org.apache.commons.collections4.bidimap.DualTreeBidiMap;
import org.sosy_lab.common.ShutdownManager;
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
import org.sosy_lab.java_smt.api.NumeralFormula.IntegerFormula;
import org.sosy_lab.java_smt.api.SolverContext.ProverOptions;

import de.monticore.lang.gdl._ast.ASTGame;
import de.monticore.lang.gdl._ast.ASTGameDistinct;
import de.monticore.lang.gdl._ast.ASTGameDoes;
import de.monticore.lang.gdl._ast.ASTGameExpression;
import de.monticore.lang.gdl._ast.ASTGameExpressionBuilder;
import de.monticore.lang.gdl._ast.ASTGameFunction;
import de.monticore.lang.gdl._ast.ASTGameFunctionBuilder;
import de.monticore.lang.gdl._ast.ASTGameFunctionDefinition;
import de.monticore.lang.gdl._ast.ASTGameFunctionDefinitionBuilder;
import de.monticore.lang.gdl._ast.ASTGameFunctionHeadBuilder;
import de.monticore.lang.gdl._ast.ASTGameInference;
import de.monticore.lang.gdl._ast.ASTGameInferenceBuilder;
import de.monticore.lang.gdl._ast.ASTGameInit;
import de.monticore.lang.gdl._ast.ASTGameLegal;
import de.monticore.lang.gdl._ast.ASTGameNext;
import de.monticore.lang.gdl._ast.ASTGameNot;
import de.monticore.lang.gdl._ast.ASTGameRelation;
import de.monticore.lang.gdl._ast.ASTGameToken;
import de.monticore.lang.gdl._ast.ASTGameTrue;
import de.monticore.lang.gdl._ast.ASTGameType;
import de.monticore.lang.gdl._ast.ASTGameValue;
import de.monticore.lang.gdl._ast.ASTGameValueBuilder;
import de.monticore.lang.gdl._symboltable.GameFunctionDefinitionSymbol;
import de.monticore.lang.gdl._symboltable.IGDLArtifactScope;

public class Interpreter {

    private final ASTGame game;
    private final IGDLArtifactScope scope;
    private BidiMap<String, Integer> constValueMap;

    private SolverContext context;

    private IntegerFormulaManager imgr;
    private BooleanFormulaManager bmgr;

    private BooleanFormula moveConstraint, modelConstraint;

    private List<ASTGameExpression> gameState, constantDefinitions;
    private List<ASTGameFunctionDefinition> legalDefinitions;
    private List<ASTGameFunctionDefinition> nextDefinitions;

    public Interpreter(ASTGame game, IGDLArtifactScope scope) {
        this.game = game;
        this.scope = scope;
    }

    public Interpreter init() throws IOException, InvalidConfigurationException, InterruptedException, SolverException {
        constValueMap = new DualTreeBidiMap<>();

        Configuration config = Configuration.defaultConfiguration();
        LogManager logger = BasicLogManager.create(config);
        ShutdownManager shutdown = ShutdownManager.create();

        context = SolverContextFactory.createSolverContext(config, logger, shutdown.getNotifier(),
                Solvers.SMTINTERPOL);

        imgr = context.getFormulaManager().getIntegerFormulaManager();
        bmgr = context.getFormulaManager().getBooleanFormulaManager();

        modelConstraint = moveConstraint = bmgr.makeTrue();

        initGameState();
        initLegalDefinitions();
        initNextDefinitions();

        return this;
    }

    private void initGameState() {
        gameState = game.getGameExpressionList()
                .stream()
                .filter(expr -> expr.getType() instanceof ASTGameInit)
                .map(expr -> (ASTGameExpression) expr.getArguments(0))
                .collect(Collectors.toList());
        constantDefinitions = game.getGameExpressionList()
                .stream()
                .filter(expr -> expr.getType() instanceof ASTGameFunction)
                .collect(Collectors.toList());
    }

    private void initLegalDefinitions() {
        legalDefinitions = game.getGameExpressionList()
                .stream()
                .filter(expr -> expr.getType() instanceof ASTGameInference)
                .filter(expr -> ((ASTGameExpression) expr.getArguments(0)).getType() instanceof ASTGameLegal)
                .map(expr -> {
                    ASTGameExpression legalExpression = (ASTGameExpression) expr.getArguments(0);
                    ASTGameRelation roleParam = legalExpression.getArguments(0);
                    ASTGameExpression moveExpression = (ASTGameExpression) legalExpression.getArguments(1);
                    List<ASTGameExpression> body = expr.getArgumentsList()
                            .stream()
                            .skip(1)
                            .map(rel -> (ASTGameExpression) rel)
                            .collect(Collectors.toList());
                    
                    ASTGameFunctionDefinition legalDef = new ASTGameFunctionDefinitionBuilder()
                            .setGameInference(
                                new ASTGameInferenceBuilder()
                                        .build()
                            )
                            .setHead(
                                new ASTGameFunctionHeadBuilder()
                                        .setName("legal")
                                        .addParameters(roleParam)
                                        .addAllParameters(moveExpression.getArgumentsList())
                                        .build()
                            )
                            .addAllBody(body)
                            .build();
                    
                    return legalDef;
                })
                .collect(Collectors.toList());
    }

    private void initNextDefinitions() {
        nextDefinitions = game.getGameExpressionList()
                .stream()
                .filter(expr -> expr.getType() instanceof ASTGameInference)
                .filter(expr -> ((ASTGameExpression) expr.getArguments(0)).getType() instanceof ASTGameNext)
                .map(expr -> {
                    ASTGameExpression nextExpression = (ASTGameExpression) expr.getArguments(0);
                    ASTGameExpression funcExpression = (ASTGameExpression) nextExpression.getArguments(0);

                    ASTGameValue funcParam = new ASTGameValueBuilder()
                        .setValue(((ASTGameFunction) funcExpression.getType()).getFunction())
                        .build();

                    List<ASTGameExpression> body = expr.getArgumentsList()
                            .stream()
                            .skip(1)
                            .map(rel -> (ASTGameExpression) rel)
                            .collect(Collectors.toList());
                    
                    ASTGameFunctionDefinition nextDef = new ASTGameFunctionDefinitionBuilder()
                            .setGameInference(
                                new ASTGameInferenceBuilder()
                                        .build()
                            )
                            .setHead(
                                new ASTGameFunctionHeadBuilder()
                                        .setName("next")
                                        .addParameters(funcParam)
                                        .addAllParameters(funcExpression.getArgumentsList())
                                        .build()
                            )
                            .addAllBody(body)
                            .build();
                    
                    return nextDef;
                })
                .collect(Collectors.toList());
    }

    private BooleanFormula buildFunction(String functionScope, List<IntegerFormula> parameters, int recursionDepth, String path, boolean isNegated, BooleanFormula upperFormula) {
        // figure out if function, legal, state or const call
        boolean isFunction = !scope.getGameFunctionDefinitionSymbols().get(functionScope).isEmpty();
        if (isFunction || functionScope.equals("legal") || functionScope.equals("next")) {
            // function
            return buildFunctionExpression(functionScope, parameters, recursionDepth, path, isNegated, upperFormula);
        } else {
            // const state
            return buildConstantStateExpression(functionScope, parameters, recursionDepth, path, isNegated);
        }
    }

    private BooleanFormula buildFunctionExpression(String functionScope, List<IntegerFormula> parameters, int recursionDepth, String path, boolean isNegated, BooleanFormula upperFormula) {
        // expand with legal functions
        List<ASTGameFunctionDefinition> allFunctionDefinitions;
        if (functionScope.equals("legal")) {
            allFunctionDefinitions = legalDefinitions;
        } else if (functionScope.equals("next")) {
            allFunctionDefinitions = nextDefinitions;
        } else {
            List<GameFunctionDefinitionSymbol> allFuncDefSymbols = scope.getGameFunctionDefinitionSymbols().get(functionScope);
            allFunctionDefinitions = allFuncDefSymbols.stream().map(symbol -> symbol.getAstNode()).collect(Collectors.toList());
        }

        // System.out.println(path + ": " + functionScope);

        BooleanFormula allOverloadsFormula = bmgr.makeFalse();

        for (int overloadId = 0; overloadId < allFunctionDefinitions.size(); overloadId++) {
            ASTGameFunctionDefinition overload = allFunctionDefinitions.get(overloadId);
            BooleanFormula overloadFormula = bmgr.makeTrue();
            String overloadPath = path + "_" + overloadId;

            // map arguments
            List<ASTGameRelation> arguments = overload.getHead().getParametersList();

            if (arguments.size() != parameters.size()) {
                continue;
            }

            BooleanFormula argumentMap = mapParametersToArguments(parameters, arguments, functionScope, recursionDepth, overloadPath);
            overloadFormula = bmgr.and(overloadFormula, argumentMap);

            // conjungate body
            List<ASTGameExpression> body = overload.getBodyList();
            for (int bodyExpressionId = 0; bodyExpressionId < body.size(); bodyExpressionId++) {
                ASTGameExpression bodyExpression = body.get(bodyExpressionId);
                BooleanFormula bodyFormula = buildBodyExpression(bodyExpression, bodyExpressionId, overloadPath, functionScope, parameters, recursionDepth, path, isNegated, bmgr.and(upperFormula, overloadFormula));

                // System.out.println("back in " + path + ": " + functionScope);

                overloadFormula = bmgr.and(overloadFormula, bodyFormula);
                // check satisfiability
                // stop as soon as not satisfiable
                if (checkSatisfiable(upperFormula, overloadFormula) == isNegated) {
                    // System.out.println("Body not satisfiable!");
                    break;
                }
            }

            allOverloadsFormula = bmgr.or(allOverloadsFormula, overloadFormula);
            
            // check satisfiability
            // stop as soon as satisfiable
            if (checkSatisfiable(upperFormula, allOverloadsFormula) != isNegated) {
                // System.out.println("overload true:\t" + functionScope + "\tBody Size: " + body.size());
                break;
            }
        }

        
        return allOverloadsFormula;
    }

    private BooleanFormula buildBodyExpression(ASTGameExpression bodyExpression, int bodyExpressionId, String overloadPath, String functionScope, List<IntegerFormula> parameters, int recursionDepth, String path, boolean isNegated, BooleanFormula upperFormula) {
        // make body expressions
        ASTGameType bodyExpressionType = bodyExpression.getType();
        if (bodyExpressionType instanceof ASTGameFunction) {
            // any recursive call (no keywords)
            String nextFunctionScope = ((ASTGameFunction) bodyExpressionType).getFunction();
            List<IntegerFormula> nextParameters = bodyExpression.getArgumentsList()
                    .stream()
                    .map(rel -> makeVariable(rel, functionScope, recursionDepth, overloadPath))
                    .collect(Collectors.toList());
            int nextRecursionDepth = recursionDepth + 1;
            String nextPath = overloadPath + "_" + bodyExpressionId + "_" + functionScope;

            return buildFunction(nextFunctionScope, nextParameters, nextRecursionDepth, nextPath, isNegated, upperFormula);
        } else if (bodyExpressionType instanceof ASTGameTrue) {
            // true keyword -> build game state expression
            ASTGameExpression gameStateExpression = (ASTGameExpression) bodyExpression.getArguments(0);

            String nextFunctionScope = ((ASTGameFunction) gameStateExpression.getType()).getFunction();
            List<IntegerFormula> nextParameters = gameStateExpression.getArgumentsList()
                    .stream()
                    .map(rel -> makeVariable(rel, functionScope, recursionDepth, overloadPath))
                    .collect(Collectors.toList());
            int nextRecursionDepth = recursionDepth + 1;
            String nextPath = overloadPath + "_" + bodyExpressionId + "_" + functionScope;

            return buildGameStateExpression(nextFunctionScope, nextParameters, nextRecursionDepth, nextPath, isNegated);
        } else if (bodyExpressionType instanceof ASTGameDistinct) {
            // distinct keyword -> build distinct expression
            IntegerFormula leftParam = makeVariable(bodyExpression.getArguments(0), functionScope, recursionDepth, overloadPath);
            IntegerFormula rightParam = makeVariable(bodyExpression.getArguments(1), functionScope, recursionDepth, overloadPath);
            
            return bmgr.not(imgr.equal(leftParam, rightParam));
        } else if (bodyExpressionType instanceof ASTGameDoes) {
            ASTGameRelation roleParam = bodyExpression.getArguments(0);
            IntegerFormula roleFormula = makeVariable(roleParam, functionScope, recursionDepth, path);

            ASTGameExpression moveExpression = (ASTGameExpression) bodyExpression.getArguments(1);
            List<IntegerFormula> moveFormulas = moveExpression.getArgumentsList()
                    .stream()
                    .map(rel -> makeVariable(rel, functionScope, recursionDepth, overloadPath))
                    .collect(Collectors.toList());
            
            BooleanFormula doesFormula = imgr.equal(roleFormula, imgr.makeVariable("move_arg_role"));
            for (int i = 0; i < moveFormulas.size(); i++) {
                BooleanFormula equality = imgr.equal(moveFormulas.get(i), imgr.makeVariable("move_arg_" + i));
                doesFormula = bmgr.and(doesFormula, equality);
            }
            return doesFormula;
        } else if (bodyExpressionType instanceof ASTGameNot) {
            ASTGameExpression innerExpression = (ASTGameExpression) bodyExpression.getArguments(0);
            return bmgr.not(buildBodyExpression(innerExpression, bodyExpressionId, overloadPath, functionScope, parameters, recursionDepth, path, !isNegated, upperFormula));
        } else if (bodyExpressionType instanceof ASTGameLegal) {
            // map legal to artificial legal funcDef
            ASTGameRelation roleParam = bodyExpression.getArguments(0);
            IntegerFormula roleFormula = makeVariable(roleParam, functionScope, recursionDepth, path);

            ASTGameExpression moveExpression = (ASTGameExpression) bodyExpression.getArguments(1);

            List<IntegerFormula> moveParameters = moveExpression.getArgumentsList()
                    .stream()
                    .map(rel -> makeVariable(rel, functionScope, recursionDepth, overloadPath))
                    .collect(Collectors.toList());
            List<IntegerFormula> nextParameters = new ArrayList<>(moveParameters.size() + 1);
            nextParameters.add(roleFormula);
            nextParameters.addAll(moveParameters);
            int nextRecursionDepth = recursionDepth + 1;
            String nextPath = overloadPath + "_" + bodyExpressionId + "_" + functionScope;

            return buildFunction("legal", nextParameters, nextRecursionDepth, nextPath, isNegated, upperFormula);
        } else {
            throw new IllegalStateException();
        }
    }

    private BooleanFormula buildGameStateExpression(String functionScope, List<IntegerFormula> parameters, int recursionDepth, String path, boolean isNegated) {
        List<ASTGameExpression> gameStateDefinitionOverloads = gameState
                .stream()
                .filter(expr -> ((ASTGameFunction) expr.getType()).getFunction().equals(functionScope))
                .collect(Collectors.toList());

        BooleanFormula allOverloadsFormula = bmgr.makeFalse();

        for (int overloadId = 0; overloadId < gameStateDefinitionOverloads.size(); overloadId++) {
            ASTGameExpression overload = gameStateDefinitionOverloads.get(overloadId);
            BooleanFormula overloadFormula = bmgr.makeTrue();
            String overloadPath = path + "_" + overloadId;

            // map arguments
            List<ASTGameRelation> arguments = overload.getArgumentsList();
            BooleanFormula argumentMap = mapParametersToArguments(parameters, arguments, functionScope, recursionDepth, overloadPath);
            overloadFormula = bmgr.and(overloadFormula, argumentMap);

            allOverloadsFormula = bmgr.or(allOverloadsFormula, overloadFormula);
        }
        return allOverloadsFormula;
    }

    private BooleanFormula buildConstantStateExpression(String functionScope, List<IntegerFormula> parameters, int recursionDepth, String path, boolean isNegated) {
        List<ASTGameExpression> constantDefinitionOverloads = constantDefinitions
                .stream()
                .filter(expr -> ((ASTGameFunction) expr.getType()).getFunction().equals(functionScope))
                .collect(Collectors.toList());

        BooleanFormula allOverloadsFormula = bmgr.makeFalse();

        for (int overloadId = 0; overloadId < constantDefinitionOverloads.size(); overloadId++) {
            ASTGameExpression overload = constantDefinitionOverloads.get(overloadId);
            BooleanFormula overloadFormula = bmgr.makeTrue();
            String overloadPath = path + "_" + overloadId;

            // map arguments
            List<ASTGameRelation> arguments = overload.getArgumentsList();
            BooleanFormula argumentMap = mapParametersToArguments(parameters, arguments, functionScope, recursionDepth, overloadPath);
            overloadFormula = bmgr.and(overloadFormula, argumentMap);

            allOverloadsFormula = bmgr.or(allOverloadsFormula, overloadFormula);
        }
        return allOverloadsFormula;
    }

    private BooleanFormula mapParametersToArguments(List<IntegerFormula> parameters, List<ASTGameRelation> arguments, String functionScope, int recursionDepth, String path) {
        BooleanFormula argumentMap = bmgr.makeTrue();
        for (int i = 0; i < parameters.size(); i++) {
            IntegerFormula parameter = parameters.get(i);
            IntegerFormula argument = makeVariable(arguments.get(i), functionScope, recursionDepth, path);
            BooleanFormula map = imgr.equal(parameter, argument);

            argumentMap = bmgr.and(argumentMap, map);
        }
        return argumentMap;
    }

    private IntegerFormula makeVariable(ASTGameRelation parameter, String functionScope, int recursionDepth, String path) {
        if (parameter instanceof ASTGameValue) {
            return imgr.makeNumber(getIntegerValue(((ASTGameValue) parameter).getValue()));
        } else if (parameter instanceof ASTGameToken) {
            return imgr.makeVariable(recursionDepth + "_func_var_" + path + "_" + functionScope + "_" + ((ASTGameToken) parameter).getToken());
        } else {
            throw new IllegalStateException("ILLEGAL STATE: Parameter must be a value or a token!");
        }
    }

    public Integer getIntegerValue(String constValue) {
        Integer constInt;
        if ((constInt = constValueMap.get(constValue)) != null) {
            return constInt;
        }
        constValueMap.put(constValue, (constInt = constValueMap.size()));
        return constInt;
    }

    private boolean checkSatisfiable(BooleanFormula... formulas) {
        ProverEnvironment prover = context.newProverEnvironment(ProverOptions.GENERATE_MODELS);

        try {
            prover.addConstraint(modelConstraint);
            prover.addConstraint(moveConstraint);
            for (BooleanFormula formula : formulas)
                prover.addConstraint(formula);
            
            boolean unsat = prover.isUnsat();
            prover.close();
            return !unsat;
        } catch (InterruptedException | SolverException e) {
            e.printStackTrace();
        }

        return false;
    }

    public List<List<String>> getAllModels(String function, List<IntegerFormula> args) {
        List<List<String>> allModels = new ArrayList<>();
        
        boolean satisfiable;
        do {
            BooleanFormula formula = buildFunction(function, args, 0, function, false, bmgr.makeTrue());
            ProverEnvironment prover = context.newProverEnvironment(ProverOptions.GENERATE_MODELS);

            try {
                // prover.addConstraint(formula);
                // prover.addConstraint(modelConstraint);
                // prover.addConstraint(moveConstraint);
                formula = bmgr.and(formula, modelConstraint);
                formula = bmgr.and(formula, moveConstraint);
                prover.addConstraint(formula);
                // System.out.println(formula);
                
                satisfiable = !prover.isUnsat();

                if (satisfiable) {
                    Model model = prover.getModel();

                    BooleanFormula modelFormula = bmgr.makeTrue();
                    List<String> assignments = new ArrayList<>(args.size());
                    
                    for (IntegerFormula arg : args) {
                        String value = constValueMap.getKey(((BigInteger) model.evaluate(arg)).intValue());
                        assignments.add(value);
                        if (value != null) {
                            modelFormula = bmgr.and(modelFormula, imgr.equal(arg, imgr.makeNumber(getIntegerValue(value))));
                        } else {
                            System.out.println("Model: Unknown Variable assigned!");
                        }
                    }
    
                    // System.out.println("Model: " + assignments);
                    allModels.add(assignments);
                    modelConstraint = bmgr.and(bmgr.not(modelFormula), modelConstraint);
                    model.close();
                }
                prover.close();
            } catch (InterruptedException | SolverException e) {
                e.printStackTrace();
                satisfiable = false;
            }
        } while(satisfiable);

        modelConstraint = bmgr.makeTrue();

        return allModels;
    }

    private void setMove(Command move) {
        if (move == null) {
            moveConstraint = bmgr.makeTrue();
            return;
        }

        moveConstraint = imgr.equal(imgr.makeVariable("move_arg_role"), imgr.makeNumber(getIntegerValue(move.getPlayer())));

        for (int i = 0; i < move.getArguments().size(); i++) {
            moveConstraint = bmgr.and(moveConstraint, imgr.equal(imgr.makeVariable("move_arg_" + i), imgr.makeNumber(getIntegerValue(move.getArguments().get(i)))));
        }
    }

    private boolean isLegal(Command move) {
        List<IntegerFormula> args = new ArrayList<>(move.getArguments().size() + 1);
        args.add(imgr.makeNumber(getIntegerValue(move.getPlayer())));
        args.addAll(
            move.getArguments().stream().map(arg -> imgr.makeNumber(getIntegerValue(arg))).collect(Collectors.toList())
        );
        BooleanFormula formula = buildFunction("legal", args, 0, "legal", false, bmgr.makeTrue());

        return checkSatisfiable(formula);
    }

    private void updateGameState() {
        List<List<String>> allStates = new LinkedList<>();
        List<Integer> distinctNextOverloadLengths = nextDefinitions.stream()
            .map(func -> func.getHead().getParametersList().size())
            .distinct()
            .collect(Collectors.toList());

        for (int overloadLength : distinctNextOverloadLengths) {
            // call function next
            String function = "next";
            // create parameters
            List<IntegerFormula> args = IntStream
                .range(0, overloadLength)
                .mapToObj(i -> "next_arg_" + i)
                .map(s -> imgr.makeVariable(s))
                .collect(Collectors.toList());

            // calculate models
            allStates.addAll(getAllModels(function, args));
        }

        // reformat strings to game state tuples
        gameState = allStates.stream()
            .map(tuple -> {
                ASTGameFunction type = new ASTGameFunctionBuilder().setFunction(tuple.get(0)).build();

                return new ASTGameExpressionBuilder()
                    .setType(type)
                    .addAllArguments(
                        tuple.stream()
                            .skip(1)
                            .map(s -> new ASTGameValueBuilder().setValue(s).build())
                            .collect(Collectors.toList())
                    )
                    .build();
            })
            .collect(Collectors.toList());
    }

    public List<ASTGameExpression> interpret(String line) {
        Command move = Command.createMoveFromLine(line);
        if (move == null) {
            // System.out.println("Move is illegal");
            return null;
        }
        return interpret(move);
    }

    public List<ASTGameExpression> interpret(Command move) {
        setMove(move);
        if (isLegal(move)) {
            updateGameState();
            return gameState;
        }
        return null;
    }

    public List<ASTGameExpression> getGameState() {
        return gameState;
    }

    public IntegerFormulaManager getImgr() {
        return imgr;
    }

    public BooleanFormulaManager getBmgr() {
        return bmgr;
    }

}
