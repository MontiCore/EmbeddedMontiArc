package de.monticore.lang.gdl.visitors;

import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.List;
import java.util.function.Consumer;

import de.monticore.ast.ASTNode;
import de.monticore.lang.gdl.GDLMill;
import de.monticore.lang.gdl._ast.ASTGame;
import de.monticore.lang.gdl._ast.ASTGameConstruct;
import de.monticore.lang.gdl._ast.ASTGameDigits;
import de.monticore.lang.gdl._ast.ASTGameInference;
import de.monticore.lang.gdl._ast.ASTGameInit;
import de.monticore.lang.gdl._ast.ASTGameKeyword;
import de.monticore.lang.gdl._ast.ASTGameLegal;
import de.monticore.lang.gdl._ast.ASTGameNext;
import de.monticore.lang.gdl._ast.ASTGameNoop;
import de.monticore.lang.gdl._ast.ASTGameRangeType;
import de.monticore.lang.gdl._ast.ASTGameSees;
import de.monticore.lang.gdl._ast.ASTGameToken;
import de.monticore.lang.gdl._ast.ASTGameTuple;
import de.monticore.lang.gdl._ast.ASTGameType;
import de.monticore.lang.gdl._ast.ASTGameValue;
import de.monticore.lang.gdl._ast.ASTGameValueType;
import de.monticore.lang.gdl._visitor.GDLHandler;
import de.monticore.lang.gdl._visitor.GDLTraverser;
import de.monticore.lang.gdl._visitor.GDLVisitor2;
import de.monticore.literals.mccommonliterals._visitor.MCCommonLiteralsVisitor2;
import de.monticore.prettyprint.IndentPrinter;

public class TypeTemplatePrinter extends IndentPrinter implements GDLVisitor2, MCCommonLiteralsVisitor2, GDLHandler {

    private static final String TEMPLATE_PREFIX_STATE = "gdl_template_state";
    private static final String TEMPLATE_PREFIX_ACTION = "gdl_template_action";


    private static final String VALUE_PREFIX = "value";
    private static final String DIGITS_POSITIVE_PREFIX = "numpos";
    private static final String DIGITS_NEGATIVE_PREFIX = "numneg";
    private static final String VALUE_TYPE_PREFIX = "value";

    private static final String PREFIX_SEPARATOR = "_";
    private static final String ELEMENT_SEPARATOR = ", ";

    private GDLTraverser traverser;

    private final String util;

    public TypeTemplatePrinter() {
        this.util = loadUtil();

        this.traverser = GDLMill.traverser();
        this.traverser.add4GDL(this);
        this.traverser.add4MCCommonLiterals(this);
        this.traverser.setGDLHandler(this);
    }

    private String loadUtil() {
        InputStream stream = PrologPrinter.class.getResourceAsStream("type_util.pl");
        try {
            BufferedReader reader = new BufferedReader(new InputStreamReader(stream, "UTF-8"));
            return reader.lines().reduce("", (s1, s2) -> s1 + "\n" + s2) + "\n";
        } catch (Exception e) {
            e.printStackTrace();
            System.err.println("Unable to load util.pl.");
            return null;
        }
    }

    @Override
    public void setTraverser(GDLTraverser traverser) {
        this.traverser = traverser;
    }

    @Override
    public GDLTraverser getTraverser() {
        return this.traverser;
    }

    @Override
    public String getContent() {
        final String content = super.getContent();

        StringBuilder contentBuilder = new StringBuilder();

        if (this.util != null) {
            contentBuilder.append("% ================================ \n");
            contentBuilder.append("% =========== TYPE UTIL ========== \n");
            contentBuilder.append("% ================================ \n");
            contentBuilder.append(this.util);
            contentBuilder.append("\n");
        }

        contentBuilder.append("% ================================ \n");
        contentBuilder.append("% ======== TYPE GENERATED ======== \n");
        contentBuilder.append("% ================================ \n");
        contentBuilder.append("\n");
        contentBuilder.append(content);
        contentBuilder.append("\n");

        return contentBuilder.toString();
    }

    private <E extends ASTNode> void printElementsSeparated(List<E> elements, int start, String separator, boolean breakLine, Consumer<E> printer) {
        for (int i = start; i < elements.size(); i++) {
            E element = elements.get(i);
            printer.accept(element);

            if (i + 1 < elements.size()) {
                if (breakLine) println(separator);
                else print(separator);
            }
        }
    }

    @Override
    public void handle(ASTGame game) {
        printElementsSeparated(game.getTuplesList(), 0, "", false, this::handleRoot);
    }

    private void handleRoot(ASTGameTuple tuple) {
        final ASTGameConstruct first = tuple.getElement(0);

        if (first instanceof ASTGameInference) {
            handleRootKeyword(tuple, (ASTGameInference) first);
        } else if (first instanceof ASTGameInit) {
            handleTemplate(tuple);
        }
    }

    private void handleRootKeyword(ASTGameTuple tuple, ASTGameInference keyword) {
        final ASTGameConstruct sentenceImplied = tuple.getElement(1);

        if (sentenceImplied instanceof ASTGameTuple) {
            handleTemplate((ASTGameTuple) sentenceImplied);
        }
    }

    public void handleTemplate(ASTGameTuple tuple) {
        ASTGameConstruct first = tuple.getElement(0);

        if (first instanceof ASTGameKeyword) {
            handleTemplateKeyword(tuple, (ASTGameKeyword) first);
        }
    }

    public void handleTemplateKeyword(ASTGameTuple tuple, ASTGameKeyword keyword) {
        if (keyword instanceof ASTGameInit) {
            handleTemplateKeyword(tuple, (ASTGameInit) keyword);
        } else if (keyword instanceof ASTGameNext) {
            handleTemplateKeyword(tuple, (ASTGameNext) keyword);
        } else if (keyword instanceof ASTGameLegal) {
            handleTemplateKeyword(tuple, (ASTGameLegal) keyword);
        }
    }

    public void handleTemplateKeyword(ASTGameTuple tuple, ASTGameInit keyword) {
        ASTGameConstruct innerTuple = tuple.getElement(1);
        handleDecideHidden(innerTuple);
    }

    public void handleTemplateKeyword(ASTGameTuple tuple, ASTGameNext keyword) {
        ASTGameConstruct innerTuple = tuple.getElement(1);
        handleDecideHidden(innerTuple);
    }

    public void handleTemplateKeyword(ASTGameTuple tuple, ASTGameLegal keyword) {
        if (tuple.getElement(1) instanceof ASTGameValue && ((ASTGameValue) tuple.getElement(1)).getValue().equals("random")) {
            // skip random
            return;
        }
        print(TEMPLATE_PREFIX_ACTION);
        print("(");
        tuple.getElement(2).accept(getTraverser());
        println(").");
    }

    public void handleDecideHidden(ASTGameConstruct construct) {
        if (construct instanceof ASTGameTuple) {
            ASTGameTuple tuple = (ASTGameTuple) construct;

            if (tuple.getElement(0) instanceof ASTGameSees) {
                if (tuple.getElement(1) instanceof ASTGameValue && ((ASTGameValue) tuple.getElement(1)).getValue().equals("none")) {
                    // skip none
                    return;
                }
                print(TEMPLATE_PREFIX_STATE);
                print("(");
                printElementsSeparated(tuple.getElementList(), 2, ELEMENT_SEPARATOR, false,  n -> n.accept(getTraverser()));
                println(").");
            } else {
                print(TEMPLATE_PREFIX_STATE);
                print("(");
                construct.accept(getTraverser());
                println(").");
            }
        } else {
            print(TEMPLATE_PREFIX_STATE);
            print("(");
            construct.accept(getTraverser());
            println(").");
        }
    }
    
    @Override
    public void handle(ASTGameTuple tuple) {
        if (tuple.isPresentType()) {
            handleTypedConstant(tuple.getType(), tuple);
        } else {
            print("[");
            printElementsSeparated(tuple.getElementList(), 0, ELEMENT_SEPARATOR, false, n -> n.accept(getTraverser()));
            print("]");
        }
    }

    @Override
    public void handle(ASTGameValue node) {
        if (node.isPresentType()) {
            handleTypedConstant(node.getType(), node);
        } else {
            handleConstantType(node);
        }
    }

    @Override
    public void handle(ASTGameDigits node) {
        if (node.isPresentType()) {
            handleTypedConstant(node.getType(), node);
        } else {
            handleConstantType(node);
        }
    }

    @Override
    public void handle(ASTGameToken node) {
        if (!node.isPresentType()) {
            System.out.println("Type expected but not found: " + node.get_SourcePositionStart());
        }
        node.getType().accept(getTraverser());
    }

    private void handleTypedConstant(ASTGameType type, ASTGameConstruct construct) {
        type.accept(getTraverser());
    }

    private void handleConstant(ASTGameConstruct constant) {
        if (constant instanceof ASTGameTuple) {
            handleConstant((ASTGameTuple) constant);
        } else if (constant instanceof ASTGameValue) {
            handleConstant((ASTGameValue) constant);
        } else if (constant instanceof ASTGameDigits) {
            handleConstant((ASTGameDigits) constant);
        } else if (constant instanceof ASTGameNoop) {
            handleConstant((ASTGameNoop) constant);
        }
    }

    private void handleConstant(ASTGameTuple constant) {
        print("[");
        printElementsSeparated(constant.getElementList(), 0, ELEMENT_SEPARATOR, false, this::handleConstant);
        print("]");
    }

    private void handleConstant(ASTGameValue constant) {
        print(VALUE_PREFIX);
        print(PREFIX_SEPARATOR);
        print(constant.getValue());
    }

    private void handleConstant(ASTGameDigits constant) {
        printNumber(constant.getNumber().getValue());
    }

    private void handleConstant(ASTGameNoop constant) {
        print("noop");
    }

    private void printNumber(int number) {
        final String numberString;
        if (number < 0) {
            numberString = DIGITS_NEGATIVE_PREFIX + PREFIX_SEPARATOR + (number*-1);
        } else {
            numberString = DIGITS_POSITIVE_PREFIX + PREFIX_SEPARATOR + number;
        }
        print(numberString);
    }

    // TYPES

    private void handleConstantType(ASTGameConstruct value) {
        print("(constant");
        print(ELEMENT_SEPARATOR);
        handleConstant(value);
        print(")");
    }

    public void visit(ASTGameRangeType node) {
        print("(range");
        print(ELEMENT_SEPARATOR);
        printNumber(node.getStart().getValue());
        print(ELEMENT_SEPARATOR);
        printNumber(node.getEnd().getValue());
        print(")");
    }

    public void visit(ASTGameValueType node) {
        print(VALUE_TYPE_PREFIX);
        print(PREFIX_SEPARATOR);
        print(node.getType());
    }

    public void visit(ASTGameNoop node) {
        print("(constant, noop)");
    }
    
}
