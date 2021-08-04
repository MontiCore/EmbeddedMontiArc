package de.monticore.lang.gdl.visitors;

import java.util.LinkedList;
import java.util.List;
import java.util.stream.Collectors;

import de.monticore.lang.gdl.GDLMill;
import de.monticore.lang.gdl._ast.ASTGameDistinct;
import de.monticore.lang.gdl._ast.ASTGameDoes;
import de.monticore.lang.gdl._ast.ASTGameExpression;
import de.monticore.lang.gdl._ast.ASTGameFunction;
import de.monticore.lang.gdl._ast.ASTGameFunctionDefinition;
import de.monticore.lang.gdl._ast.ASTGameFunctionHead;
import de.monticore.lang.gdl._ast.ASTGameGoal;
import de.monticore.lang.gdl._ast.ASTGameInference;
import de.monticore.lang.gdl._ast.ASTGameInit;
import de.monticore.lang.gdl._ast.ASTGameLegal;
import de.monticore.lang.gdl._ast.ASTGameNext;
import de.monticore.lang.gdl._ast.ASTGameNot;
import de.monticore.lang.gdl._ast.ASTGameRelation;
import de.monticore.lang.gdl._ast.ASTGameRole;
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

    public PrologPrinter() {
        this.traverser = GDLMill.traverser();
        this.traverser.add4GDL(this);
        this.traverser.add4MCCommonLiterals(this);
        this.traverser.setGDLHandler(this);
    }

    public GDLTraverser getTraverser() {
        return traverser;
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
    }

    @Override
    public void visit(ASTGameValue node) {
        String value = node.getValue();
        print("value_" + value);
    }

    public void visit(ASTGameFunction node) {
        print("function_" + node.getFunction());
    }

    public void visit(ASTGameRole node) {
        print("role");
    }

    public void visit(ASTGameTrue node) {
        print("state_");
    }
    
    public void visit(ASTGameInit node) {
        print("state_");
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

    private boolean isInFunctionDefinition = false;
    @Override
    public void handle(ASTGameExpression node) {
        ASTGameType type = node.getType();

        if (isInFunctionDefinition) {
            if (type instanceof ASTGameDistinct) {
                node.getArguments(0).accept(getTraverser());
                print(" \\== ");
                node.getArguments(1).accept(getTraverser());
            } else if (type instanceof ASTGameTrue) {
                type.accept(getTraverser());
                node.getArguments(0).accept(getTraverser());
            } else if (type instanceof ASTGameDoes) {
                type.accept(getTraverser());
                print("(");
                
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
                print(")");
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
                node.getArguments(0).accept(getTraverser());
            } else if (type instanceof ASTGameInference) {
                ASTGameExpression head = (ASTGameExpression) node.getArguments(0);
                ASTGameType headType = head.getType();

                if (headType instanceof ASTGameNext) {
                    headType.accept(getTraverser());
                    print("(");
                    ASTGameExpression parameters = (ASTGameExpression) head.getArguments(0);
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
                    print(", ");

                    ASTGameExpression parameters = (ASTGameExpression) head.getArguments(1);
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
            } else {
                type.accept(getTraverser());
                print("(");
                for (int i = 0; i < node.getArgumentsList().size(); i++) {
                    node.getArguments(i).accept(getTraverser());
    
                    if (i + 1 < node.getArgumentsList().size()) {
                        print(", ");
                    }
                }
                println(").");
            }
        }
    }

    @Override
    public void handle(ASTGameFunctionDefinition node) {
        ASTGameFunctionHead astHead = node.getHead();
        String functionName = astHead.getName();

        print("function_" + functionName);
        print("(");
        for (int i = 0; i < astHead.getParametersList().size(); i++) {
            astHead.getParameters(i).accept(getTraverser());

            if (i + 1 < astHead.getParametersList().size()) {
                print(", ");
            }
        }
        print(")");
        println(" :-");
        indent();
        
        List<ASTGameExpression> distinctExpressions = new LinkedList<>();

        isInFunctionDefinition = true;
        for (int i = 0; i < node.getBodyList().size(); i++) {
            if (node.getBody(i).getType() instanceof ASTGameDistinct) {
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
