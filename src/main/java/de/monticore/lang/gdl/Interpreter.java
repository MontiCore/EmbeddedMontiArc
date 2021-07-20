package de.monticore.lang.gdl;

import java.io.IOException;
import java.math.BigInteger;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.Queue;
import java.util.LinkedList;
import java.util.Map;
import java.util.HashMap;
import java.util.function.Predicate;
import java.util.stream.Collectors;

import org.apache.commons.collections4.BidiMap;
import org.apache.commons.collections4.bidimap.DualTreeBidiMap;
import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.Pair;
// import org.sosy_lab.common.ShutdownManager;
import org.sosy_lab.common.configuration.Configuration;
import org.sosy_lab.common.configuration.InvalidConfigurationException;
import org.sosy_lab.common.log.BasicLogManager;
import org.sosy_lab.common.log.LogManager;
import org.sosy_lab.java_smt.SolverContextFactory;
import org.sosy_lab.java_smt.SolverContextFactory.Solvers;
import org.sosy_lab.java_smt.api.BooleanFormula;
import org.sosy_lab.java_smt.api.BooleanFormulaManager;
import org.sosy_lab.java_smt.api.FormulaManager;
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
import de.monticore.lang.gdl._ast.ASTGameInit;
import de.monticore.lang.gdl._ast.ASTGameLegal;
import de.monticore.lang.gdl._ast.ASTGameNot;
import de.monticore.lang.gdl._ast.ASTGameRelation;
import de.monticore.lang.gdl._ast.ASTGameToken;
import de.monticore.lang.gdl._ast.ASTGameTrue;
import de.monticore.lang.gdl._ast.ASTGameValue;
import de.monticore.lang.gdl._ast.ASTGameFunction;
import de.monticore.lang.gdl._ast.ASTGameFunctionDefinition;
import de.monticore.lang.gdl._ast.ASTGameFunctionHead;
import de.monticore.lang.gdl._ast.ASTGameInference;
import de.monticore.lang.gdl._symboltable.GameFunctionDefinitionSymbol;
import de.monticore.lang.gdl._symboltable.IGDLArtifactScope;

import de.se_rwth.commons.logging.Log;

public class Interpreter {

    private final ASTGame game;
    private final IGDLArtifactScope scope;
    private BidiMap<String, Integer> constValueMap;

    private List<ASTGameExpression> legalExpressions;

    private State volatileState;
    private State constantState;

    private BooleanFormula moveConstraint;

    IntegerFormulaManager imgr;
    BooleanFormulaManager bmgr;
    FormulaManager fmgr;

    SolverContext context;

    public Interpreter(ASTGame game, IGDLArtifactScope scope) {
        this.game = game;
        this.scope = scope;
    }

    public Interpreter init() throws IOException, InvalidConfigurationException, InterruptedException, SolverException {
        Configuration config = Configuration.defaultConfiguration();
        LogManager logger = BasicLogManager.create(config);
        ShutdownManager shutdown = ShutdownManager.create();

        context = SolverContextFactory.createSolverContext(config, logger, shutdown.getNotifier(),
                Solvers.SMTINTERPOL);

        imgr = context.getFormulaManager().getIntegerFormulaManager();
        bmgr = context.getFormulaManager().getBooleanFormulaManager();
        fmgr = context.getFormulaManager();


        // integer representation of each string constant (as bijective map)
        constValueMap = new DualTreeBidiMap<>();

        volatileState = new State(imgr, bmgr, constValueMap, "state");
        constantState = new State(imgr, bmgr, constValueMap, "constant");

        legalExpressions = new ArrayList<>();

        // all sub terms
        for (ASTGameExpression expression : game.getGameExpressionList()) {
            if (expression.getType() instanceof ASTGameInit) {
                // Is init state
                volatileState.addRelation((ASTGameExpression) expression.getArguments(0));
            } else if (expression.getType() instanceof ASTGameFunction) {
                // Is constant
                constantState.addRelation(expression);
            } else if (expression.getType() instanceof ASTGameInference
                    && expression.getArguments(0) instanceof ASTGameExpression
                    && ((ASTGameExpression) expression.getArguments(0)).getType() instanceof ASTGameLegal) {
                // remember all legal expressions
                legalExpressions.add(expression);
            }
        }

        Log.println("const: " + constantState.getConstraint() + "\n");
        Log.println("state: " + volatileState.getConstraint() + "\n\n");

        Log.println(constValueMap.toString());

        moveConstraint = bmgr.makeTrue();

        return this;
    }

    public void interpret(String line) {
        // REMOVE LATER:
        test();

        // Move move = Move.createMoveFromLine(line);
        // if (evalLegal(move)) {
        //     computeNextState(move);
        // }
    }

    private void computeNextState(Move move) {

    }

    // START

    private void test() {
        // String functionName = "isLegalMove";
        // List<IntegerFormula> parameterMappings = List.of(
        //     imgr.makeNumber(getConstRepresentation("pawn")),
        //     imgr.makeNumber(getConstRepresentation("white")),
        //     imgr.makeNumber(getConstRepresentation("a")),
        //     imgr.makeNumber(getConstRepresentation("2")),
        //     imgr.makeNumber(getConstRepresentation("a")),
        //     imgr.makeNumber(getConstRepresentation("4"))
        // );
        String functionName = "f1";
        List<IntegerFormula> parameterMappings = List.of(
            imgr.makeNumber(getConstRepresentation("1"))
        );


        BooleanFormula gigaMegaFormula = buildFunctionConstraint(functionName, parameterMappings);

        System.out.println("SAT:" + checkSatisfiable(gigaMegaFormula));

        ProverEnvironment prover = context.newProverEnvironment(ProverOptions.GENERATE_MODELS);

        try {
            prover.addConstraint(volatileState.getConstraint());
            prover.addConstraint(constantState.getConstraint());
            prover.addConstraint(moveConstraint);
            
            prover.addConstraint(gigaMegaFormula);
            
            boolean unsat = prover.isUnsat();

            if (!unsat) {
                Model model = prover.getModel();

                // System.out.println("MODEL FOR POS:" + constValueMap.getKey(((BigInteger) model.evaluate(imgr.makeVariable("pos"))).intValue()));
            }

            prover.close();
        } catch (InterruptedException | SolverException e) {
            e.printStackTrace();
        }

        List<BooleanFormula> constr = List.of(
            imgr.equal(imgr.makeNumber(getConstRepresentation("a")), imgr.makeVariable("field" + "_arg_state_" + 0)),
            imgr.equal(imgr.makeNumber(getConstRepresentation("2")), imgr.makeVariable("field" + "_arg_state_" + 1)),
            imgr.equal(imgr.makeNumber(getConstRepresentation("white_pawn")), imgr.makeVariable("field" + "_arg_state_" + 2))
        );
        
        System.out.println("SANITY:" + checkSatisfiable(bmgr.and(constr)));

        System.exit(0);
    }

    private class CallArguments {
        public final String functionName;
        public final List<IntegerFormula> parameterMappings;
        public final int recursionDepth;
        public final BooleanFormula previousStageFormula;
        public final boolean isNegated;

        public CallArguments (String functionName, List<IntegerFormula> parameterMappings, int recursionDepth, BooleanFormula previousStageFormula, boolean isNegated) {
            this.functionName = functionName;
            this.parameterMappings = parameterMappings;
            this.recursionDepth = recursionDepth;
            this.previousStageFormula = previousStageFormula;
            this.isNegated = isNegated;
        }

    }

    private BooleanFormula buildFunctionConstraint(String functionName, List<IntegerFormula> parameterMappings) {
        // init stack (function name, parameter mappings, recursion depth)
        // stack must be queue?
        Queue<CallArguments> callStack = new LinkedList<>();
        callStack.offer(new CallArguments(functionName, parameterMappings, 0, bmgr.makeTrue(), false));

        // init root formula (as satisfiable)
        BooleanFormula rootFunctionFormula = bmgr.makeTrue();

        while (!callStack.isEmpty()) {
            CallArguments current = callStack.poll();
            // current function of stack
            String currentFunctionName = current.functionName;
            List<IntegerFormula> currentParameterMappings = current.parameterMappings;
            int recursionDepth = current.recursionDepth;
            BooleanFormula previousStageFormula = current.previousStageFormula;
            boolean currentIsNegated = current.isNegated;

            Log.println("Function: " + currentFunctionName + "_" + recursionDepth);

            // Get all function definitions for function
            List<GameFunctionDefinitionSymbol> allFuncDefSymbols = scope.getGameFunctionDefinitionSymbols().get(currentFunctionName);
            Set<ASTGameFunctionDefinition> allFunctionDefinitions = allFuncDefSymbols.stream().map(symbol -> symbol.getAstNode()).collect(Collectors.toSet());

            // save all body formulas and recursive expressions for later use
            Map<ASTGameFunctionDefinition, Pair<BooleanFormula, List<ASTGameExpression>>> allFunctionBodys = new HashMap<>();

            // build all function bodys BUT recursive calls (function calls, legal calls)
            for (ASTGameFunctionDefinition functionDefinition : allFunctionDefinitions) {
                ASTGameFunctionHead functionHead = functionDefinition.getHead();

                BooleanFormula bodyFormula = bmgr.makeTrue();
                
                // recursive function calls
                List<ASTGameExpression> recursiveExpressions = functionDefinition
                    .getBodyList().stream()
                    .filter(new RecursiveExpressionsFilter())
                    .collect(Collectors.toList());
                
                // everything else
                List<ASTGameExpression> nonRecursiveExpressions = functionDefinition.getBodyList();
                nonRecursiveExpressions.removeIf(recursiveExpressions::contains);

                // map params from upper stage to current args
                for (int i = 0; i < currentParameterMappings.size(); i++) {
                    // argument from upper stage
                    IntegerFormula arg = currentParameterMappings.get(i);
                    // parameter in current function head
                    ASTGameRelation parameter = functionHead.getParameters(i);
                    IntegerFormula param = getRepresentation(parameter, functionName, recursionDepth);
                    // argument and parameter must be equal
                    bodyFormula = bmgr.and(bodyFormula, imgr.equal(param, arg));
                }

                for (ASTGameExpression bodyExpression : nonRecursiveExpressions) {
                    bodyFormula = bmgr.and(bodyFormula, buildExpression(bodyExpression, functionName, recursionDepth));
                }


                BooleanFormula temp = bodyFormula;
                if (currentIsNegated) {
                    temp = bmgr.not(temp);
                }
                
                boolean satisfiable = checkSatisfiable(rootFunctionFormula, temp, previousStageFormula);

                Log.println("overload: " + bodyFormula  + "\n");
                Log.println("satisfiable: " + satisfiable  + "\n");

                // add to current function calls
                if (satisfiable) allFunctionBodys.put(functionDefinition, new ImmutablePair<>(bodyFormula, recursiveExpressions));
            }

            // build formula (function overload = or expression)
            BooleanFormula functionOverloadFormula = bmgr.makeFalse();
            for (Pair<BooleanFormula, List<ASTGameExpression>> currentOverload : allFunctionBodys.values()) {
                functionOverloadFormula = bmgr.or(functionOverloadFormula, currentOverload.getLeft());
            }

            // use negation
            if (currentIsNegated) {
                functionOverloadFormula = bmgr.not(functionOverloadFormula);
            }
            
            // use previous stage formula here (like this ?)
            functionOverloadFormula = bmgr.and(previousStageFormula, functionOverloadFormula);

            boolean satisfiable = checkSatisfiable(rootFunctionFormula, functionOverloadFormula);

            // check if still satisfiable
            Log.println(satisfiable + "");

            if (satisfiable) {
                // add formula to root function formula
                rootFunctionFormula = bmgr.and(rootFunctionFormula, functionOverloadFormula);
                // is satisfiable -> at least one overload is satisfiable -> do stack calls
                // ?? just conjungate with root formula here or >>>use disjunctive context (func overload)<<<?
                for (Pair<BooleanFormula, List<ASTGameExpression>> currentOverload : allFunctionBodys.values()) {
                    BooleanFormula currentStageFunction = currentOverload.getLeft();
                    List<ASTGameExpression> recursiveCalls = currentOverload.getRight();

                    // Handle all recursive calls
                    for (ASTGameExpression recursiveCall : recursiveCalls) {
                        CallArguments next = buildRecursiveExpression(recursiveCall, functionName, recursionDepth, currentStageFunction, currentIsNegated);
                        callStack.offer(next);
                    }
                }
            } else {
                // is not satisfiable -> no overload is satisfiable -> abort recursion?
                rootFunctionFormula = bmgr.and(rootFunctionFormula, functionOverloadFormula);
            }

        }
        return rootFunctionFormula;
    }

    private boolean checkSatisfiable(BooleanFormula... formulas) {
        ProverEnvironment prover = context.newProverEnvironment(ProverOptions.GENERATE_MODELS);

        try {
            prover.addConstraint(volatileState.getConstraint());
            prover.addConstraint(constantState.getConstraint());
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

    private CallArguments buildRecursiveExpression(ASTGameExpression expression, String functionName, int recursionDepth, BooleanFormula previousStageFormula, boolean negated) {
        if (expression.getType() instanceof ASTGameFunction) {
            String nextFunctionName = ((ASTGameFunction) expression.getType()).getFunction();
            List<IntegerFormula> parameterMappings = expression.getArgumentsList()
                .stream()
                .map(rel -> getRepresentation(rel, functionName, recursionDepth))
                .collect(Collectors.toList());
            return new CallArguments(nextFunctionName, parameterMappings, recursionDepth + 1, previousStageFormula, negated);
        } else if (expression.getType() instanceof ASTGameLegal) {
            // TODO: find all legal expressions
            throw new UnsupportedOperationException("Not implemented");
        } else if (expression.getType() instanceof ASTGameNot) {
            ASTGameExpression innerExpression = (ASTGameExpression) expression.getArguments(0);
            return buildRecursiveExpression(innerExpression, functionName, recursionDepth, previousStageFormula, !negated);
        } else {
            throw new IllegalStateException();
        }
    }

    private BooleanFormula buildExpression(ASTGameExpression expression, String functionName, int recursionDepth) {
        if (expression.getType() instanceof ASTGameNot) {
            // is "not" expression
            // apply negation on inner expression
            return bmgr.not(buildExpression((ASTGameExpression) expression.getArguments(0), functionName, recursionDepth));
        } else if (expression.getType() instanceof ASTGameDistinct) {
            // is distinct expression
            // store left and right param
            IntegerFormula leftParam, rightParam;
            leftParam = getRepresentation(expression.getArguments(0), functionName, recursionDepth);
            rightParam = getRepresentation(expression.getArguments(1), functionName, recursionDepth);

            // left param must be unequal to right param
            return bmgr.not(imgr.equal(leftParam, rightParam));
        } else if (expression.getType() instanceof ASTGameDoes) {
            // is in "does"
            // get role param
            IntegerFormula roleParam = getRepresentation(expression.getArguments(0), functionName, recursionDepth);
            
            // set equal to move role param
            BooleanFormula result = imgr.equal(roleParam, imgr.makeVariable("move_arg_role"));

            // set other params equal to move param
            ASTGameExpression moveExpression = (ASTGameExpression) expression.getArguments(1);
            List<ASTGameRelation> parameters = moveExpression.getArgumentsList();
            for (int i = 0; i < parameters.size(); i++) {
                IntegerFormula param = getRepresentation(parameters.get(i), functionName, recursionDepth);

                // all params must be equal to matched move arg
                result = bmgr.and(result, imgr.equal(param, imgr.makeVariable("move_arg_" + i)));
            }
            return result;
        } else if (expression.getType() instanceof ASTGameTrue) {
            // is in game state
            // get inner expression
            ASTGameExpression gameStateExpression = (ASTGameExpression) expression.getArguments(0);

            // get name as state identifier
            String gameStateName = ((ASTGameFunction) gameStateExpression.getType()).getFunction();

            // get parameters and map with game state identifier
            List<ASTGameRelation> parameters = gameStateExpression.getArgumentsList();

            BooleanFormula result = bmgr.makeTrue();
            for (int i = 0; i < parameters.size(); i++) {
                IntegerFormula param = getRepresentation(parameters.get(i), functionName, recursionDepth);
                IntegerFormula gameStateArgument = imgr.makeVariable(gameStateName + "_arg_state_" + i);

                // map parameter to matching argument
                result = bmgr.and(result, imgr.equal(param, gameStateArgument));
            }
            return result;
        } else if (expression.getType() instanceof ASTGameFunction) {
            // is in game state
            // get name as state identifier
            String gameStateName = ((ASTGameFunction) expression.getType()).getFunction();

            // get parameters and map with constants identifier
            List<ASTGameRelation> parameters = expression.getArgumentsList();

            BooleanFormula result = bmgr.makeTrue();
            for (int i = 0; i < parameters.size(); i++) {
                IntegerFormula param = getRepresentation(parameters.get(i), functionName, recursionDepth);
                IntegerFormula gameStateArgument = imgr.makeVariable(gameStateName + "_arg_constant_" + i);

                // map parameter to matching argument
                result = bmgr.and(result, imgr.equal(param, gameStateArgument));
            }
            return result;
        } else {
            throw new IllegalStateException();
        }
    }

    private IntegerFormula getRepresentation(ASTGameRelation parameter, String functionName, int recursionDepth) {
        if (parameter instanceof ASTGameValue) {
            return imgr.makeNumber(getConstRepresentation(((ASTGameValue) parameter).getValue()));
        } else if (parameter instanceof ASTGameToken) {
            return imgr.makeVariable(recursionDepth + "_func_var_" + functionName + "_" + ((ASTGameToken) parameter).getToken());
        } else {
            throw new IllegalStateException("ILLEGAL STATE: Parameter must be a value or a token!");
        }
    }

    private class RecursiveExpressionsFilter implements Predicate<ASTGameExpression> {

        @Override
        public boolean test(ASTGameExpression expr) {
            return (expr.getType() instanceof ASTGameFunction)
                && !scope.getGameFunctionDefinitionSymbols().get(((ASTGameFunction) expr.getType()).getFunction()).isEmpty()
                || (expr.getType() instanceof ASTGameLegal)
                // if "not" expression, then inner expression must pass test
                || (expr.getType() instanceof ASTGameNot)
                && test((ASTGameExpression) expr.getArguments(0));
        }

    }

    // END

    private Integer getConstRepresentation(String constValue) {
        Integer constInt;
        if ((constInt = constValueMap.get(constValue)) != null) {
            return constInt;
        }
        constValueMap.put(constValue, (constInt = constValueMap.size()));
        return constInt;
    }

    /*
    private boolean evalLegal(Move move) {
        // create move constraint
        IntegerFormula constFormula = imgr.makeNumber(getConstRepresentation(move.getPlayer()));
        IntegerFormula varFormula = imgr.makeVariable("move_arg_role");
        BooleanFormula equality = imgr.equal(constFormula, varFormula);
        BooleanFormula moveConstraint = equality;
        for (int i = 0; i < move.getArguments().size(); i++) {
            constFormula = imgr.makeNumber(getConstRepresentation(move.getArguments().get(i)));
            varFormula = imgr.makeVariable("move_arg_" + i);
            equality = imgr.equal(constFormula, varFormula);
            moveConstraint = bmgr.and(moveConstraint, equality);
        }
        this.moveConstraint = moveConstraint;

        for (ASTGameExpression expression : game.getGameExpressionList()) {
            // check if legal expression
            if (expression.getType() instanceof ASTGameInference
                    && ((ASTGameExpression) expression.getArguments(0)).getType() instanceof ASTGameLegal) {
                // is legal expression
                ASTGameExpression legalExpression = (ASTGameExpression) expression.getArguments(0);
                ASTGameRelation player = legalExpression.getArguments(0);
                ASTGameExpression moveExpression = (ASTGameExpression) legalExpression.getArguments(1);

                // check if move arguments length matches legal expression
                if (moveExpression.getArgumentsList().size() != move.getArguments().size()) {
                    continue;
                }
            }

        }
        return false;
    }

    private List<BooleanFormula> evaluateExpression(ASTGameFunctionDefinition function, List<IntegerFormula> arguments, List<BooleanFormula> constraints, int recursionDepth) {
        ASTGameFunctionHead functionHead = (ASTGameFunctionHead) function.getHead();
        List<ASTGameRelation> parameters = functionHead.getParametersList();

        List<BooleanFormula> newConstraints = new ArrayList<>();

        if (parameters.size() != arguments.size()) {
            return null;
        }

        for (int i = 0; i < parameters.size(); i++) {
            ASTGameRelation headArgument = parameters.get(i);
            IntegerFormula givenArgument = arguments.get(i);

            IntegerFormula constOrToken;
            if (headArgument instanceof ASTGameValue) {
                ASTGameValue value = (ASTGameValue) headArgument;
                constOrToken = imgr.makeNumber(getConstRepresentation(value.getValue()));
            } else {
                ASTGameToken token = (ASTGameToken) headArgument;
                constOrToken = imgr.makeVariable("func_" + functionHead.getName() + "_" + token.getToken() + "_" + recursionDepth);
            }
            
            BooleanFormula equality = imgr.equal(givenArgument, constOrToken);

            newConstraints.add(equality);
        }

        if (isSatisfiable(List.of(constraints, newConstraints)) == null) {
            return null;
        }

        // konstanten, true, does zuerst, danach rekursive prüfung (optimierung mit zwei schleifen)

        List<ASTGameExpression> bodyExpressions = function.getBodyList()
            .stream().map(a -> (ASTGameExpression) a).collect(Collectors.toList());

        for (ASTGameExpression expression : bodyExpressions) {
            if (expression.getType() instanceof ASTGameFunction) {
                // entweder function oder constant

                String functionName = ((ASTGameFunction) expression.getType()).getFunction();
                List<GameFunctionDefinitionSymbol> symbols = scope.getGameFunctionDefinitionSymbols().get(functionName);
                if (symbols.isEmpty()) {
                    // is constant
                    for (int i = 0; i < expression.getArgumentsList().size(); i++) {
                        ASTGameRelation parameter = expression.getArguments(i);
    
                        IntegerFormula constOrToken;
                        if (parameter instanceof ASTGameValue) {
                            ASTGameValue value = (ASTGameValue) parameter;
                            constOrToken = imgr.makeNumber(getConstRepresentation(value.getValue()));
                        } else {
                            ASTGameToken token = (ASTGameToken) parameter;
                            constOrToken = imgr.makeVariable("func_" + functionHead.getName() + "_" + token.getToken() + "_" + recursionDepth);
                        }
                        IntegerFormula constant = imgr.makeVariable(functionName + "_arg_" + "constant" + "_" + i);
                        BooleanFormula equality = imgr.equal(constOrToken, constant);
    
                        newConstraints.add(equality);

                        
                    }

                    if (isSatisfiable(List.of(constraints, newConstraints)) == null) {
                        return null;
                    }


                } else {
                    // is function

                    // create list with arguments for function call
                    List<IntegerFormula> nextArguments = new ArrayList<>(expression.getArgumentsList().size());
                    for (ASTGameRelation relation : expression.getArgumentsList()) {
                        IntegerFormula constOrToken;
                        if (relation instanceof ASTGameValue) {
                            ASTGameValue value = (ASTGameValue) relation;
                            constOrToken = imgr.makeNumber(getConstRepresentation(value.getValue()));
                        } else {
                            ASTGameToken token = (ASTGameToken) relation;
                            constOrToken = imgr.makeVariable("func_" + functionHead.getName() + "_" + token.getToken() + "_" + recursionDepth);
                        }
                        nextArguments.add(constOrToken);
                    }

                    List<BooleanFormula> contraintsCopy = new ArrayList<>();
                    contraintsCopy.addAll(constraints);
                    contraintsCopy.addAll(newConstraints);
                    
                    BooleanFormula allFunctions = null;
                    // for each function implementation (overloaded function)
                    for (GameFunctionDefinitionSymbol symbol: symbols) {
                        ASTGameFunctionDefinition functionDefinition = symbol.getAstNode();
                        List<BooleanFormula> result = evaluateExpression(functionDefinition, nextArguments, contraintsCopy, recursionDepth + 1);

                        if (result != null) {
                            if (allFunctions == null) {
                                allFunctions = bmgr.and(result);
                            } else {
                                allFunctions = bmgr.or(allFunctions, bmgr.and(result));
                            }
                        }
                    }

                    if (allFunctions == null) {
                        return null;
                    } else {
                        newConstraints.add(allFunctions);
                    }
                }
            } else if (expression.getType() instanceof ASTGameTrue) {
                ASTGameExpression stateExpression = (ASTGameExpression) expression.getArguments(0);
                ASTGameFunction stateName = (ASTGameFunction) stateExpression.getType();
                
                for (int i = 0; i < stateExpression.getArgumentsList().size(); i++) {
                    ASTGameRelation parameter = stateExpression.getArguments(i);

                    IntegerFormula constOrToken;
                    if (parameter instanceof ASTGameValue) {
                        ASTGameValue value = (ASTGameValue) parameter;
                        constOrToken = imgr.makeNumber(getConstRepresentation(value.getValue()));
                    } else {
                        ASTGameToken token = (ASTGameToken) parameter;
                        constOrToken = imgr.makeVariable("func_" + functionHead.getName() + "_" + token.getToken() + "_" + recursionDepth);
                    }
                    IntegerFormula stateConstant = imgr.makeVariable(stateName.getFunction() + "_arg_" + "state" + "_" + i);
                    BooleanFormula equality = imgr.equal(constOrToken, stateConstant);

                    newConstraints.add(equality);
                }

                // return null wenn nicht mehr erfüllbar
                if (isSatisfiable(List.of(constraints, newConstraints)) == null) {
                    return null;
                }

            } else if (expression.getType() instanceof ASTGameDoes) {
                // is does expression
                ASTGameRelation playerParameter = (ASTGameRelation) expression.getArguments(0);

                // constraint for player, player must match player in move
                IntegerFormula playerConstOrToken;
                if (playerParameter instanceof ASTGameValue) {
                    ASTGameValue value = (ASTGameValue) playerParameter;
                    playerConstOrToken = imgr.makeNumber(getConstRepresentation(value.getValue()));
                } else {
                    ASTGameToken token = (ASTGameToken) playerParameter;
                    playerConstOrToken = imgr.makeVariable("func_" + functionHead.getName() + "_" + token.getToken() + "_" + recursionDepth);
                }
                IntegerFormula moveRole = imgr.makeVariable("move_arg_role");
                BooleanFormula equality = imgr.equal(playerConstOrToken, moveRole);

                // constraint for move args, all args must match move expression args
                ASTGameExpression moveExpression = (ASTGameExpression) expression.getArguments(1);
                for (int i = 0; i < moveExpression.getArgumentsList().size(); i++) {
                    ASTGameRelation parameter = moveExpression.getArguments(i);
                    IntegerFormula constOrToken;
                    if (playerParameter instanceof ASTGameValue) {
                        ASTGameValue value = (ASTGameValue) playerParameter;
                        playerConstOrToken = imgr.makeNumber(getConstRepresentation(value.getValue()));
                    } else {
                        ASTGameToken token = (ASTGameToken) playerParameter;
                        playerConstOrToken = imgr.makeVariable("func_" + functionHead.getName() + "_" + token.getToken() + "_" + recursionDepth);
                    }
                    IntegerFormula moveArg = imgr.makeVariable("move_arg_" + i);
                    equality = bmgr.and(equality, imgr.equal(playerConstOrToken, moveRole));
                }

                newConstraints.add(equality);

                if (isSatisfiable(List.of(constraints, newConstraints)) == null) {
                    return null;
                }
            } else if (expression.getType() instanceof ASTGameDistinct) {
                // is distinct expression
                ASTGameRelation leftSide = (ASTGameRelation) expression.getArguments(0);
                ASTGameRelation rightSide = (ASTGameRelation) expression.getArguments(1);

                IntegerFormula leftConstOrToken;
                if (leftSide instanceof ASTGameValue) {
                    ASTGameValue value = (ASTGameValue) leftSide;
                    leftConstOrToken = imgr.makeNumber(getConstRepresentation(value.getValue()));
                } else {
                    ASTGameToken token = (ASTGameToken) leftSide;
                    leftConstOrToken = imgr.makeVariable("func_" + functionHead.getName() + "_" + token.getToken() + "_" + recursionDepth);
                }
                IntegerFormula rightConstOrToken;
                if (rightSide instanceof ASTGameValue) {
                    ASTGameValue value = (ASTGameValue) rightSide;
                    rightConstOrToken = imgr.makeNumber(getConstRepresentation(value.getValue()));
                } else {
                    ASTGameToken token = (ASTGameToken) rightSide;
                    rightConstOrToken = imgr.makeVariable("func_" + functionHead.getName() + "_" + token.getToken() + "_" + recursionDepth);
                }

                BooleanFormula unequals = bmgr.not(imgr.equal(leftConstOrToken, rightConstOrToken));
                
                newConstraints.add(unequals);

                if (isSatisfiable(List.of(constraints, newConstraints)) == null) {
                    return null;
                }
            } else if (expression.getType() instanceof ASTGameNot) {
                
            }
        }

        return null;
    }

    private Model isSatisfiable(List<List<BooleanFormula>> constraintsLists) {
        ProverEnvironment prover = context.newProverEnvironment(ProverOptions.GENERATE_MODELS);

        try {
            prover.addConstraint(volatileState.getConstraint());
            prover.addConstraint(constantState.getConstraint());
            prover.addConstraint(moveConstraint);
            for (List<BooleanFormula> constraintsList : constraintsLists) {
                for (BooleanFormula constraint : constraintsList) {
                    prover.addConstraint(constraint);
                }
            }
            
            boolean isUnsat = prover.isUnsat();
            if (!isUnsat) {
                // generate model
                Model model = prover.getModel();
                return model;
            }
        } catch (InterruptedException | SolverException e) {
            // Log.error(e);
        }

        return null;
    }
    */

}
