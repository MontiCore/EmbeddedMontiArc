package de.monticore.lang.gdl.visitors;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

import de.monticore.lang.gdl.FunctionSignature;
import de.monticore.lang.gdl.GDLMill;
import de.monticore.lang.gdl._ast.ASTGameAdd;
import de.monticore.lang.gdl._ast.ASTGameCount;
import de.monticore.lang.gdl._ast.ASTGameDigits;
import de.monticore.lang.gdl._ast.ASTGameDistinct;
import de.monticore.lang.gdl._ast.ASTGameDiv;
import de.monticore.lang.gdl._ast.ASTGameDoes;
import de.monticore.lang.gdl._ast.ASTGameEqual;
import de.monticore.lang.gdl._ast.ASTGameExpression;
import de.monticore.lang.gdl._ast.ASTGameFunction;
import de.monticore.lang.gdl._ast.ASTGameFunctionDefinition;
import de.monticore.lang.gdl._ast.ASTGameFunctionHead;
import de.monticore.lang.gdl._ast.ASTGameGoal;
import de.monticore.lang.gdl._ast.ASTGameGreater;
import de.monticore.lang.gdl._ast.ASTGameInference;
import de.monticore.lang.gdl._ast.ASTGameInit;
import de.monticore.lang.gdl._ast.ASTGameLegal;
import de.monticore.lang.gdl._ast.ASTGameLess;
import de.monticore.lang.gdl._ast.ASTGameMod;
import de.monticore.lang.gdl._ast.ASTGameMult;
import de.monticore.lang.gdl._ast.ASTGameNext;
import de.monticore.lang.gdl._ast.ASTGameNot;
import de.monticore.lang.gdl._ast.ASTGameNumber;
import de.monticore.lang.gdl._ast.ASTGameRelation;
import de.monticore.lang.gdl._ast.ASTGameRole;
import de.monticore.lang.gdl._ast.ASTGameSees;
import de.monticore.lang.gdl._ast.ASTGameSub;
import de.monticore.lang.gdl._ast.ASTGameSucc;
import de.monticore.lang.gdl._ast.ASTGameTerminal;
import de.monticore.lang.gdl._ast.ASTGameToken;
import de.monticore.lang.gdl._ast.ASTGameTrue;
import de.monticore.lang.gdl._ast.ASTGameType;
import de.monticore.lang.gdl._ast.ASTGameValue;
import de.monticore.lang.gdl._visitor.GDLHandler;
import de.monticore.lang.gdl._visitor.GDLTraverser;
import de.monticore.lang.gdl._visitor.GDLVisitor2;
import de.monticore.literals.mccommonliterals._visitor.MCCommonLiteralsVisitor2;
import de.monticore.prettyprint.IndentPrinter;

public class PrologPrinter extends IndentPrinter implements GDLVisitor2, MCCommonLiteralsVisitor2, GDLHandler {
    private GDLTraverser traverser;
    private Set<FunctionSignature> functionSignatures = new HashSet<>();
    private Set<FunctionSignature> legalSignatures = new HashSet<>();

    private Set<FunctionSignature> statesSignatures = new HashSet<>();
    private Set<FunctionSignature> hiddenStatesSignatures = new HashSet<>();
    private Set<FunctionSignature> nextSignatures = new HashSet<>();
    private Set<FunctionSignature> hiddenNextSignatures = new HashSet<>();
    private boolean hasTerminal = false;

    private Set<String> roles = new HashSet<>();

    public PrologPrinter() {
        this.traverser = GDLMill.traverser();
        this.traverser.add4GDL(this);
        this.traverser.add4MCCommonLiterals(this);
        this.traverser.setGDLHandler(this);
    }

    private boolean saveVars = false;
    private List<String> vars;
    private String uniqueID = "";

    private void saveVars(boolean save) {
        this.saveVars = save;
        if (save) {
            vars = new LinkedList<>();
        }
    }

    public GDLTraverser getTraverser() {
        return traverser;
    }

    public Set<FunctionSignature> getFunctionSignatures() {
        return functionSignatures;
    }

    public Set<FunctionSignature> getLegalSignatures() {
        return legalSignatures;
    }

    public Set<FunctionSignature> getStatesSignatures() {
        return statesSignatures;
    }

    public Set<FunctionSignature> getHiddenStatesSignatures() {
        return hiddenStatesSignatures;
    }

    public Set<FunctionSignature> getNextSignatures() {
        return nextSignatures;
    }

    public Set<FunctionSignature> getHiddenNextSignatures() {
        return hiddenNextSignatures;
    }

    public boolean hasTerminal() {
        return hasTerminal;
    }

    public boolean hasRandom() {
        return roles.contains("random");
    }

    public Set<String> getRoles() {
        return roles;
    }

    public void setTraverser(GDLTraverser traverser) {
        this.traverser = traverser;
    }

    @Override
    public void visit(ASTGameNot node) {
        print("\\+ ");
    }

    @Override
    public void visit(ASTGameToken node) {
        String token = node.getToken();
        print("Token_" + token);
        if (saveVars) vars.add("Token_" + token);
    }

    @Override
    public void visit(ASTGameValue node) {
        String value = node.toString();
        print("value_" + value);
        if (saveVars) vars.add("value_" + value);
    }

    @Override
    public void visit(ASTGameDigits node) {
        String prefix = node.getVal().isNegative() ? "valnn_" : "value_";
        String value = node.toString();
        if (node.getVal().isNegative()) value = value.substring(1);
        print(prefix + value);
        if (saveVars) vars.add(prefix + value);
    }

    public void visit(ASTGameFunction node) {
        print("function_" + node.getFunction());
    }

    public void visit(ASTGameRole node) {
        print("role");
    }

    public void visit(ASTGameTrue node) {
        print("state");
    }
    
    public void visit(ASTGameInit node) {
        print("state");
    }
    
    public void visit(ASTGameSees node) {
        print("_hidden");
    }

    public void visit(ASTGameNext node) {
        print("function_next");
    }

    public void visit(ASTGameLegal node) {
        print("function_legal");
    }

    public void visit(ASTGameDoes node) {
        print("input");
    }

    public void visit(ASTGameTerminal node) {
        print("function_terminal");
    }

    public void visit(ASTGameGoal node) {
        print("function_goal");
    }

    public String getPlaceholderStates() {
        StringBuilder sb = new StringBuilder();
        sb.append("state(placeholder).\n");
        sb.append("state(placeholder, placeholder).\n");
        sb.append("state_hidden(placeholder, placeholder).\n");
        sb.append("state_hidden(placeholder, placeholder, placeholder).\n");
        return sb.toString();
    }

    private boolean isInFunctionDefinition = false;
    @Override
    public void handle(ASTGameExpression node) {
        ASTGameType type = node.getType();

        if (isInFunctionDefinition) {
            if (type instanceof ASTGameDistinct) {
                int numArguments = node.getArgumentsList().size();
                //Itterate over all Arguments
                for (int i = 0; i < numArguments-1; i++)
                {
                    for (int j = i+1; j<numArguments; j++)
                    {
                        node.getArguments(i).accept(getTraverser());
                        print(" \\== ");
                        node.getArguments(j).accept(getTraverser());
                        // More Inequalities following
                        if(j != numArguments - 1)
                        {
                            print(", ");
                        }
                    }
                    // More Inequalities following
                    if(i != numArguments - 2)
                    {
                        print(", ");
                    }
                }
                //Code for 2 Arguments only
                /* node.getArguments(0).accept(getTraverser());
                print(" \\== ");
                node.getArguments(1).accept(getTraverser());*/ 
            } else if (type instanceof ASTGameAdd) {
                print("add(");
                node.getArguments(0).accept(getTraverser());
                print(", ");
                node.getArguments(1).accept(getTraverser());
                print(", ");
                node.getArguments(2).accept(getTraverser());
                print(")");
            } else if (type instanceof ASTGameSub) {
                print("sub(");
                node.getArguments(0).accept(getTraverser());
                print(", ");
                node.getArguments(1).accept(getTraverser());
                print(", ");
                node.getArguments(2).accept(getTraverser());
                print(")");
            } else if (type instanceof ASTGameMult) {
                print("mult(");
                node.getArguments(0).accept(getTraverser());
                print(", ");
                node.getArguments(1).accept(getTraverser());
                print(", ");
                node.getArguments(2).accept(getTraverser());
                print(")");
            } else if (type instanceof ASTGameDiv) {
                print("div(");
                node.getArguments(0).accept(getTraverser());
                print(", ");
                node.getArguments(1).accept(getTraverser());
                print(", ");
                node.getArguments(2).accept(getTraverser());
                print(")");
            } else if (type instanceof ASTGameSucc) {
                print("succ(");
                node.getArguments(0).accept(getTraverser());
                print(", ");
                node.getArguments(1).accept(getTraverser());
                print(")");
            } else if (type instanceof ASTGameLess) {
                print("less(");
                node.getArguments(0).accept(getTraverser());
                print(", ");
                node.getArguments(1).accept(getTraverser());
                print(")");
            } else if (type instanceof ASTGameGreater) {
                print("greater(");
                node.getArguments(0).accept(getTraverser());
                print(", ");
                node.getArguments(1).accept(getTraverser());
                print(")");
            } else if (type instanceof ASTGameEqual) {
                print("equal(");
                node.getArguments(0).accept(getTraverser());
                print(", ");
                node.getArguments(1).accept(getTraverser());
                print(")");
            } else if (type instanceof ASTGameNumber) {
                print("num(");
                node.getArguments(0).accept(getTraverser());
                print(")");
            } else if (type instanceof ASTGameMod) {
                print("modulo(");
                node.getArguments(0).accept(getTraverser());
                print(", ");
                node.getArguments(1).accept(getTraverser());
                print(", ");
                node.getArguments(2).accept(getTraverser());
                print(")");
            } else if (type instanceof ASTGameTrue) {
                type.accept(getTraverser());

                ASTGameExpression innerExpression = (ASTGameExpression) node.getArguments(0);
                if (innerExpression.getType() instanceof ASTGameSees) {
                    // handle hidden states
                    ASTGameExpression seesExpression = innerExpression;
                    innerExpression = (ASTGameExpression) seesExpression.getArguments(1);

                    seesExpression.getType().accept(getTraverser());
                    
                    print("(");
                    innerExpression.getType().accept(getTraverser());
                    print(", ");
                    seesExpression.getArguments(0).accept(getTraverser());

                    if (innerExpression.getArgumentsList().size() > 0) {
                        print(", (");
                        for (int i = 0; i < innerExpression.getArgumentsList().size(); i++) {
                            innerExpression.getArguments(i).accept(getTraverser());

                            if (i < innerExpression.getArgumentsList().size() - 1) {
                                print(", ");
                            }
                        }
                        print(")");
                    }
                    
                    print(")");
                } else {
                    print("(");
                    innerExpression.getType().accept(getTraverser());
                    
                    if (innerExpression.getArgumentsList().size() > 0) {
                        print(", (");
                        for (int i = 0; i < innerExpression.getArgumentsList().size(); i++) {
                            innerExpression.getArguments(i).accept(getTraverser());

                            if (i < innerExpression.getArgumentsList().size() - 1) {
                                print(", ");
                            }
                        }
                        print(")");
                    }
                    
                    print(")");
                }
            } else if (type instanceof ASTGameDoes) {
                type.accept(getTraverser());
                print("((");
                
                ASTGameRelation astPlayer = node.getArguments(0);
                astPlayer.accept(getTraverser());
                print(", ");

                ASTGameExpression parameters = (ASTGameExpression) node.getArguments(1);
                String pseudoValue = "value_" + ((ASTGameFunction) parameters.getType()).getFunction();
                print(pseudoValue);

                for (int i = 0; i < parameters.getArgumentsList().size(); i++) {
                    if (i == 0) {
                        print(", ");
                    }

                    parameters.getArguments(i).accept(getTraverser());

                    if (i + 1 < parameters.getArgumentsList().size()) {
                        print(", ");
                    }
                }
                print("))");
            } else if(type instanceof ASTGameCount) {
                StringBuilder buffer = new StringBuilder();
                Set<String> varSet = new HashSet<>();
                for (int i = 1; i < node.getArgumentsList().size(); i++) {
                    PrologPrinter temp = new PrologPrinter();
                    temp.saveVars(true);
                    temp.isInFunctionDefinition = true;
                    temp.uniqueID = uniqueID + "_" + i;

                    node.getArguments(i).accept(temp.getTraverser());

                    buffer.append(temp.getContent());
                    if (i < node.getArgumentsList().size() - 1) {
                        buffer.append(", ");
                    }
                    varSet.addAll(temp.vars);
                }

                println("setof(");
                indent();
                // model tuple:
                print("(");
                List<String> varList = new LinkedList<>(varSet);
                for (int i = 0; i < varList.size(); i++) {
                    print(varList.get(i));
                    if (i < varList.size() - 1) {
                        print(", ");
                    }
                }
                println("),");

                // inner expression
                println("(" + buffer.toString() + "),");

                println("Models" + uniqueID);
                unindent();
                println("),");

                println("length(Models" + uniqueID + ", Length" + uniqueID + "),");

                print("number_to_atom(Length" + uniqueID + ", ");
                node.getArguments(0).accept(getTraverser());
                print(")");
            } else if (type instanceof ASTGameLegal) {
                type.accept(getTraverser());
                print("(");
                ASTGameRelation astPlayer = node.getArguments(0);
                astPlayer.accept(getTraverser());
                print(", (");

                ASTGameExpression parameters = (ASTGameExpression) node.getArguments(1);
                String pseudoValue = "value_" + ((ASTGameFunction) parameters.getType()).getFunction();
                print(pseudoValue);

                for (int i = 0; i < parameters.getArgumentsList().size(); i++) {
                    if (i == 0) {
                        print(", ");
                    }

                    parameters.getArguments(i).accept(getTraverser());

                    if (i + 1 < parameters.getArgumentsList().size()) {
                        print(", ");
                    }
                }

                print("))");
            } else {
                type.accept(getTraverser());
                print("(");
                for (int i = 0; i < node.getArgumentsList().size(); i++) {
                    node.getArguments(i).accept(getTraverser());
    
                    if (i + 1 < node.getArgumentsList().size()) {
                        print(", ");
                    }
                }
                print(")");
            }
        } else {
            if (type instanceof ASTGameInit) {
                type.accept(getTraverser());

                ASTGameExpression innerExpression = (ASTGameExpression) node.getArguments(0);
                if (innerExpression.getType() instanceof ASTGameSees) {
                    // handle hidden states
                    ASTGameExpression seesExpression = innerExpression;
                    innerExpression = (ASTGameExpression) seesExpression.getArguments(1);

                    seesExpression.getType().accept(getTraverser());
                    
                    print("(");
                    innerExpression.getType().accept(getTraverser());
                    print(", ");
                    seesExpression.getArguments(0).accept(getTraverser());

                    if (innerExpression.getArgumentsList().size() > 0) {
                        print(", (");
                        for (int i = 0; i < innerExpression.getArgumentsList().size(); i++) {
                            innerExpression.getArguments(i).accept(getTraverser());

                            if (i < innerExpression.getArgumentsList().size() - 1) {
                                print(", ");
                            }
                        }
                        print(")");
                    }
                    
                    println(").");
                    

                    String state = ((ASTGameFunction) innerExpression.getType()).getFunction();
                    int arity = innerExpression.getArgumentsList().size();
                    FunctionSignature s = new FunctionSignature(state, arity + 1);
                    hiddenStatesSignatures.add(s);
                } else {
                    print("(");
                    innerExpression.getType().accept(getTraverser());

                    if (innerExpression.getArgumentsList().size() > 0) {
                        print(", (");
                        for (int i = 0; i < innerExpression.getArgumentsList().size(); i++) {
                            innerExpression.getArguments(i).accept(getTraverser());

                            if (i < innerExpression.getArgumentsList().size() - 1) {
                                print(", ");
                            }
                        }
                        print(")");
                    }
                    
                    println(").");

                    // add to state signatures
                    String state = ((ASTGameFunction) innerExpression.getType()).getFunction();
                    int arity = innerExpression.getArgumentsList().size();
                    FunctionSignature s = new FunctionSignature(state, arity);
                    statesSignatures.add(s);
                }
            } else if (type instanceof ASTGameInference) {
                ASTGameExpression head = (ASTGameExpression) node.getArguments(0);
                ASTGameType headType = head.getType();

                if (headType instanceof ASTGameNext) {
                    headType.accept(getTraverser());
                    ASTGameExpression parameters = (ASTGameExpression) head.getArguments(0);

                    if (parameters.getType() instanceof ASTGameSees) {
                        print("_hidden(");

                        ASTGameExpression seesExpression = parameters;
                        parameters = (ASTGameExpression) seesExpression.getArguments(1);

                        seesExpression.getArguments(0).accept(getTraverser());
                        print(", ");

                        // add hiddenNextSignature
                        String functionName = "function_next_hidden";
                        int arity = parameters.getArgumentsList().size() + 2;
                        FunctionSignature s = new FunctionSignature(functionName, arity);
                        hiddenNextSignatures.add(s);
                    } else {
                        // add nextSignature
                        String functionName = "function_next";
                        int arity = parameters.getArgumentsList().size() + 1;
                        FunctionSignature s = new FunctionSignature(functionName, arity);
                        nextSignatures.add(s);

                        print("(");
                    }

                    String pseudoValue = "value_" + ((ASTGameFunction) parameters.getType()).getFunction();
                    print(pseudoValue);

                    for (int i = 0; i < parameters.getArgumentsList().size(); i++) {
                        if (i == 0) {
                            print(", ");
                        }

                        parameters.getArguments(i).accept(getTraverser());

                        if (i + 1 < parameters.getArgumentsList().size()) {
                            print(", ");
                        }
                    }

                    println(") :-");
                    indent();

                    List<ASTGameExpression> distinctExpressions = new LinkedList<>();

                    isInFunctionDefinition = true;
                    for (int i = 1; i < node.getArgumentsList().size(); i++) {
                        ASTGameExpression bodyExpression = (ASTGameExpression) node.getArguments(i);
                        if (bodyExpression.getType() instanceof ASTGameDistinct) {
                            distinctExpressions.add(bodyExpression);
                            continue;
                        } else if (bodyExpression.getType() instanceof ASTGameAdd) {
                            distinctExpressions.add(bodyExpression);
                            continue;
                        } else if (bodyExpression.getType() instanceof ASTGameSub) {
                            distinctExpressions.add(bodyExpression);
                            continue;
                        } else if (bodyExpression.getType() instanceof ASTGameMult) {
                            distinctExpressions.add(bodyExpression);
                            continue;
                        } else if (bodyExpression.getType() instanceof ASTGameDiv) {
                            distinctExpressions.add(bodyExpression);
                            continue;
                        } else if (bodyExpression.getType() instanceof ASTGameSucc) {
                            distinctExpressions.add(bodyExpression);
                            continue;
                        } else if (bodyExpression.getType() instanceof ASTGameLess) {
                            distinctExpressions.add(bodyExpression);
                            continue;
                        } else if (bodyExpression.getType() instanceof ASTGameGreater) {
                            distinctExpressions.add(bodyExpression);
                            continue;
                        } else if (bodyExpression.getType() instanceof ASTGameEqual) {
                            distinctExpressions.add(bodyExpression);
                            continue;
                        } else if (bodyExpression.getType() instanceof ASTGameNumber) {
                            distinctExpressions.add(bodyExpression);
                            continue;
                        } else if (bodyExpression.getType() instanceof ASTGameMod) {
                            distinctExpressions.add(bodyExpression);
                            continue;
                        }
                        bodyExpression.accept(getTraverser());

                        if (i + 1 < node.getArgumentsList().size() || !distinctExpressions.isEmpty()) {
                            println(",");
                        }
                    }
                    for (int i = 0; i < distinctExpressions.size(); i++) {
                        distinctExpressions.get(i).accept(getTraverser());

                        if (i + 1 < distinctExpressions.size()) {
                            println(",");
                        }
                    }

                    isInFunctionDefinition = false;
                    println(".");
                    unindent();
                } else if (headType instanceof ASTGameLegal) {
                    headType.accept(getTraverser());
                    print("(");
                    ASTGameRelation astPlayer = head.getArguments(0);
                    astPlayer.accept(getTraverser());
                    print(", (");

                    ASTGameExpression parameters = (ASTGameExpression) head.getArguments(1);
                    String pseudoValue = "value_" + ((ASTGameFunction) parameters.getType()).getFunction();
                    print(pseudoValue);

                    int arity = parameters.getArgumentsList().size() + 1;
                    FunctionSignature s = new FunctionSignature("legal", arity + 1);
                    legalSignatures.add(s);

                    for (int i = 0; i < parameters.getArgumentsList().size(); i++) {
                        if (i == 0) {
                            print(", ");
                        }

                        parameters.getArguments(i).accept(getTraverser());

                        if (i + 1 < parameters.getArgumentsList().size()) {
                            print(", ");
                        }
                    }

                    println(")) :-");
                    indent();

                    List<ASTGameExpression> distinctExpressions = new LinkedList<>();

                    isInFunctionDefinition = true;
                    for (int i = 1; i < node.getArgumentsList().size(); i++) {
                        ASTGameExpression bodyExpression = (ASTGameExpression) node.getArguments(i);
                        if (bodyExpression.getType() instanceof ASTGameDistinct) {
                            distinctExpressions.add(bodyExpression);
                            continue;
                        } else if (bodyExpression.getType() instanceof ASTGameAdd) {
                            distinctExpressions.add(bodyExpression);
                            continue;
                        } else if (bodyExpression.getType() instanceof ASTGameSub) {
                            distinctExpressions.add(bodyExpression);
                            continue;
                        } else if (bodyExpression.getType() instanceof ASTGameMult) {
                            distinctExpressions.add(bodyExpression);
                            continue;
                        } else if (bodyExpression.getType() instanceof ASTGameDiv) {
                            distinctExpressions.add(bodyExpression);
                            continue;
                        } else if (bodyExpression.getType() instanceof ASTGameSucc) {
                            distinctExpressions.add(bodyExpression);
                            continue;
                        } else if (bodyExpression.getType() instanceof ASTGameLess) {
                            distinctExpressions.add(bodyExpression);
                            continue;
                        } else if (bodyExpression.getType() instanceof ASTGameGreater) {
                            distinctExpressions.add(bodyExpression);
                            continue;
                        } else if (bodyExpression.getType() instanceof ASTGameEqual) {
                            distinctExpressions.add(bodyExpression);
                            continue;
                        } else if (bodyExpression.getType() instanceof ASTGameNumber) {
                            distinctExpressions.add(bodyExpression);
                            continue;
                        } else if (bodyExpression.getType() instanceof ASTGameMod) {
                            distinctExpressions.add(bodyExpression);
                            continue;
                        }
                        bodyExpression.accept(getTraverser());

                        if (i + 1 < node.getArgumentsList().size() || !distinctExpressions.isEmpty()) {
                            println(",");
                        }
                    }
                    for (int i = 0; i < distinctExpressions.size(); i++) {
                        distinctExpressions.get(i).accept(getTraverser());

                        if (i + 1 < distinctExpressions.size()) {
                            println(",");
                        }
                    }

                    println(".");
                    unindent();

                    isInFunctionDefinition = false;
                } else if (headType instanceof ASTGameGoal) {
                    // add functionSignature
                    String functionName = "goal";
                    int arity = head.getArgumentsList().size();

                    FunctionSignature signature = new FunctionSignature(functionName, arity);
                    functionSignatures.add(signature);


                    headType.accept(getTraverser());
                    
                    print("(");

                    for (int i = 0; i < head.getArgumentsList().size(); i++) {
                        head.getArguments(i).accept(getTraverser());

                        if (i + 1 < head.getArgumentsList().size()) {
                            print(", ");
                        }
                    }

                    println(") :-");
                    indent();

                    List<ASTGameExpression> distinctExpressions = new LinkedList<>();

                    isInFunctionDefinition = true;
                    for (int i = 1; i < node.getArgumentsList().size(); i++) {
                        ASTGameExpression bodyExpression = (ASTGameExpression) node.getArguments(i);
                        if (bodyExpression.getType() instanceof ASTGameDistinct) {
                            distinctExpressions.add(bodyExpression);
                            continue;
                        } else if (bodyExpression.getType() instanceof ASTGameAdd) {
                            distinctExpressions.add(bodyExpression);
                            continue;
                        } else if (bodyExpression.getType() instanceof ASTGameSub) {
                            distinctExpressions.add(bodyExpression);
                            continue;
                        } else if (bodyExpression.getType() instanceof ASTGameMult) {
                            distinctExpressions.add(bodyExpression);
                            continue;
                        } else if (bodyExpression.getType() instanceof ASTGameDiv) {
                            distinctExpressions.add(bodyExpression);
                            continue;
                        } else if (bodyExpression.getType() instanceof ASTGameSucc) {
                            distinctExpressions.add(bodyExpression);
                            continue;
                        } else if (bodyExpression.getType() instanceof ASTGameLess) {
                            distinctExpressions.add(bodyExpression);
                            continue;
                        } else if (bodyExpression.getType() instanceof ASTGameGreater) {
                            distinctExpressions.add(bodyExpression);
                            continue;
                        } else if (bodyExpression.getType() instanceof ASTGameEqual) {
                            distinctExpressions.add(bodyExpression);
                            continue;
                        } else if (bodyExpression.getType() instanceof ASTGameNumber) {
                            distinctExpressions.add(bodyExpression);
                            continue;
                        } else if (bodyExpression.getType() instanceof ASTGameMod) {
                            distinctExpressions.add(bodyExpression);
                            continue;
                        }
                        bodyExpression.accept(getTraverser());

                        if (i + 1 < node.getArgumentsList().size() || !distinctExpressions.isEmpty()) {
                            println(",");
                        }
                    }
                    for (int i = 0; i < distinctExpressions.size(); i++) {
                        distinctExpressions.get(i).accept(getTraverser());

                        if (i + 1 < distinctExpressions.size()) {
                            println(",");
                        }
                    }

                    isInFunctionDefinition = false;
                    println(".");
                    unindent();
                }
            } else if (type instanceof ASTGameTerminal) {
                // add functionSignature
                functionSignatures.add(new FunctionSignature("terminal", 0));
                hasTerminal = true;

                type.accept(getTraverser());

                println("() :-");
                indent();

                List<ASTGameExpression> distinctExpressions = new LinkedList<>();

                isInFunctionDefinition = true;
                for (int i = 0; i < node.getArgumentsList().size(); i++) {
                    ASTGameExpression bodyExpression = (ASTGameExpression) node.getArguments(i);
                    if (bodyExpression.getType() instanceof ASTGameDistinct) {
                        distinctExpressions.add(bodyExpression);
                        continue;
                    } else if (bodyExpression.getType() instanceof ASTGameAdd) {
                        distinctExpressions.add(bodyExpression);
                        continue;
                    } else if (bodyExpression.getType() instanceof ASTGameSub) {
                        distinctExpressions.add(bodyExpression);
                        continue;
                    } else if (bodyExpression.getType() instanceof ASTGameMult) {
                        distinctExpressions.add(bodyExpression);
                        continue;
                    } else if (bodyExpression.getType() instanceof ASTGameDiv) {
                        distinctExpressions.add(bodyExpression);
                        continue;
                    } else if (bodyExpression.getType() instanceof ASTGameSucc) {
                        distinctExpressions.add(bodyExpression);
                        continue;
                    } else if (bodyExpression.getType() instanceof ASTGameLess) {
                        distinctExpressions.add(bodyExpression);
                        continue;
                    } else if (bodyExpression.getType() instanceof ASTGameGreater) {
                        distinctExpressions.add(bodyExpression);
                        continue;
                    } else if (bodyExpression.getType() instanceof ASTGameEqual) {
                        distinctExpressions.add(bodyExpression);
                        continue;
                    } else if (bodyExpression.getType() instanceof ASTGameNumber) {
                        distinctExpressions.add(bodyExpression);
                        continue;
                    } else if (bodyExpression.getType() instanceof ASTGameMod) {
                        distinctExpressions.add(bodyExpression);
                        continue;
                    }
                    bodyExpression.accept(getTraverser());

                    if (i + 1 < node.getArgumentsList().size() || !distinctExpressions.isEmpty()) {
                        println(",");
                    }
                }
                for (int i = 0; i < distinctExpressions.size(); i++) {
                    distinctExpressions.get(i).accept(getTraverser());

                    if (i + 1 < distinctExpressions.size()) {
                        println(",");
                    }
                }

                isInFunctionDefinition = false;
                println(".");
                unindent();

            } else if(type instanceof ASTGameRole) {
                // type game role
                type.accept(getTraverser());
                print("(");
                for (int i = 0; i < node.getArgumentsList().size(); i++) {
                    node.getArguments(i).accept(getTraverser());

                    // add role to role set
                    this.roles.add(((ASTGameValue) node.getArguments(i)).getValue());
    
                    if (i + 1 < node.getArgumentsList().size()) {
                        print(", ");
                    }
                }
                println(").");
            } else {
                // constant
                type.accept(getTraverser());
                print("(");
                for (int i = 0; i < node.getArgumentsList().size(); i++) {
                    node.getArguments(i).accept(getTraverser());
    
                    if (i + 1 < node.getArgumentsList().size()) {
                        print(", ");
                    }
                }
                println(").");

                // add functionSignature
                if (type instanceof ASTGameFunction) {
                    String functionName = ((ASTGameFunction) type).getFunction();
                    int arity = node.getArgumentsList().size();

                    FunctionSignature signature = new FunctionSignature(functionName, arity);
                    functionSignatures.add(signature);
                }
            }
        }
    }

    @Override
    public void handle(ASTGameFunctionDefinition node) {
        ASTGameFunctionHead astHead = node.getHead();
        String functionName = astHead.getName();

        // add functionSignature
        int arity = astHead.getParametersList().size();

        FunctionSignature signature = new FunctionSignature(functionName, arity);
        functionSignatures.add(signature);


        print("function_" + functionName);
        print("(");
        for (int i = 0; i < astHead.getParametersList().size(); i++) {
            astHead.getParameters(i).accept(getTraverser());

            if (i + 1 < astHead.getParametersList().size()) {
                print(", ");
            }
        }
        print(")");

        if (node.getBodyList().size() == 0) {
            println(".");
            return;
        }

        println(" :-");
        indent();
        
        List<ASTGameExpression> distinctExpressions = new LinkedList<>();

        isInFunctionDefinition = true;
        for (int i = 0; i < node.getBodyList().size(); i++) {
            if (node.getBody(i).getType() instanceof ASTGameDistinct) {
                distinctExpressions.add(node.getBody(i));
                continue;
            } else if (node.getBody(i).getType() instanceof ASTGameAdd) {
                distinctExpressions.add(node.getBody(i));
                continue;
            } else if (node.getBody(i).getType() instanceof ASTGameSub) {
                distinctExpressions.add(node.getBody(i));
                continue;
            } else if (node.getBody(i).getType() instanceof ASTGameMult) {
                distinctExpressions.add(node.getBody(i));
                continue;
            } else if (node.getBody(i).getType() instanceof ASTGameDiv) {
                distinctExpressions.add(node.getBody(i));
                continue;
            } else if (node.getBody(i).getType() instanceof ASTGameSucc) {
                distinctExpressions.add(node.getBody(i));
                continue;
            } else if (node.getBody(i).getType() instanceof ASTGameLess) {
                distinctExpressions.add(node.getBody(i));
                continue;
            } else if (node.getBody(i).getType() instanceof ASTGameGreater) {
                distinctExpressions.add(node.getBody(i));
                continue;
            } else if (node.getBody(i).getType() instanceof ASTGameEqual) {
                distinctExpressions.add(node.getBody(i));
                continue;
            } else if (node.getBody(i).getType() instanceof ASTGameNumber) {
                distinctExpressions.add(node.getBody(i));
                continue;
            } else if (node.getBody(i).getType() instanceof ASTGameMod) {
                distinctExpressions.add(node.getBody(i));
                continue;
            }
            node.getBody(i).accept(getTraverser());

            if (i + 1 < node.getBodyList().size() || !distinctExpressions.isEmpty()) {
                println(",");
            }
        }
        for (int i = 0; i < distinctExpressions.size(); i++) {
            distinctExpressions.get(i).accept(getTraverser());

            if (i + 1 < distinctExpressions.size()) {
                println(",");
            }
        }

        isInFunctionDefinition = false;
        println(".");
        unindent();
    }

}
