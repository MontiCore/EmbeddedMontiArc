package de.monticore.lang.gdl.visitors;

import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.List;
import java.util.function.Consumer;

import de.monticore.ast.ASTNode;
import de.monticore.lang.gdl.GDLMill;
import de.monticore.lang.gdl._ast.ASTGame;
import de.monticore.lang.gdl._ast.ASTGameAdd;
import de.monticore.lang.gdl._ast.ASTGameConstruct;
import de.monticore.lang.gdl._ast.ASTGameCount;
import de.monticore.lang.gdl._ast.ASTGameDigits;
import de.monticore.lang.gdl._ast.ASTGameDistinct;
import de.monticore.lang.gdl._ast.ASTGameDiv;
import de.monticore.lang.gdl._ast.ASTGameDoes;
import de.monticore.lang.gdl._ast.ASTGameEqual;
import de.monticore.lang.gdl._ast.ASTGameGoal;
import de.monticore.lang.gdl._ast.ASTGameGreater;
import de.monticore.lang.gdl._ast.ASTGameInference;
import de.monticore.lang.gdl._ast.ASTGameInit;
import de.monticore.lang.gdl._ast.ASTGameKeyword;
import de.monticore.lang.gdl._ast.ASTGameLegal;
import de.monticore.lang.gdl._ast.ASTGameLess;
import de.monticore.lang.gdl._ast.ASTGameMod;
import de.monticore.lang.gdl._ast.ASTGameMult;
import de.monticore.lang.gdl._ast.ASTGameNext;
import de.monticore.lang.gdl._ast.ASTGameNoop;
import de.monticore.lang.gdl._ast.ASTGameNot;
import de.monticore.lang.gdl._ast.ASTGameNumber;
import de.monticore.lang.gdl._ast.ASTGameRole;
import de.monticore.lang.gdl._ast.ASTGameSees;
import de.monticore.lang.gdl._ast.ASTGameSub;
import de.monticore.lang.gdl._ast.ASTGameSucc;
import de.monticore.lang.gdl._ast.ASTGameTerminal;
import de.monticore.lang.gdl._ast.ASTGameToken;
import de.monticore.lang.gdl._ast.ASTGameTrue;
import de.monticore.lang.gdl._ast.ASTGameTuple;
import de.monticore.lang.gdl._ast.ASTGameTypeCombineDef;
import de.monticore.lang.gdl._ast.ASTGameTypeDef;
import de.monticore.lang.gdl._ast.ASTGameTypeMapDef;
import de.monticore.lang.gdl._ast.ASTGameValue;
import de.monticore.lang.gdl._visitor.GDLHandler;
import de.monticore.lang.gdl._visitor.GDLTraverser;
import de.monticore.lang.gdl._visitor.GDLVisitor2;
import de.monticore.literals.mccommonliterals._ast.ASTSignedNatLiteral;
import de.monticore.literals.mccommonliterals._visitor.MCCommonLiteralsVisitor2;
import de.monticore.prettyprint.IndentPrinter;

public class PrologPrinter extends IndentPrinter implements GDLVisitor2, MCCommonLiteralsVisitor2, GDLHandler {

    private static final String RULE_PREFIX = "gdl_rule";

    private static final String VALUE_PREFIX = "value";
    private static final String TOKEN_PREFIX = "Token";
    private static final String DIGITS_POSITIVE_PREFIX = "numpos";
    private static final String DIGITS_NEGATIVE_PREFIX = "numneg";
    
    private static final String PREFIX_SEPARATOR = "_";
    private static final String ELEMENT_SEPARATOR = ", ";

    private static final String INDENT_STRING = "    ";

    private final String util;

    private GDLTraverser traverser;

    public PrologPrinter() {
        this.util = loadUtil();

        this.traverser = GDLMill.traverser();
        this.traverser.add4GDL(this);
        this.traverser.add4MCCommonLiterals(this);
        this.traverser.setGDLHandler(this);
    }

    private String loadUtil() {
        InputStream stream = PrologPrinter.class.getResourceAsStream("util.pl");
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
            contentBuilder.append("% ============= UTIL ============= \n");
            contentBuilder.append("% ================================ \n");
            contentBuilder.append(this.util);
            contentBuilder.append("\n");
        }

        contentBuilder.append("% ================================ \n");
        contentBuilder.append("% =========== GENERATED ========== \n");
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

    // FIX INDENT

    int indent = 0;
    boolean newLine = true;

    @Override
    public void indent() {
        indent++;
    }

    @Override
    public void unindent() {
        indent--;
    }

    @Override
    public void print(Object o) {
        if (newLine) {
            super.print(INDENT_STRING.repeat(indent));
            super.print(o.toString());
            newLine = false;
        }
        else super.print(o);
    }

    @Override
    public void println(Object o) {
        if (newLine) {
            super.print(INDENT_STRING.repeat(indent));
            super.println(o.toString());
        }
        else super.println(o);
        newLine = true;
    }

    // END OF FIX INDENT

    @Override
    public void handle(ASTGame game) {
        // translate each tuple
        printElementsSeparated(game.getTuplesList(), 0, "", true, this::handleRoot);
    }

    private void handleRoot(ASTGameTuple tuple) {
        final ASTGameConstruct first = tuple.getElement(0);

        if (first instanceof ASTGameKeyword) {
            handleRootKeyword(tuple, (ASTGameKeyword) first);
        } else {
            handleRootFact(tuple);
        }
    }

    private void handleRootKeyword(ASTGameTuple tuple, ASTGameKeyword keyword) {
        if (keyword instanceof ASTGameInference) {
            handleRootKeyword(tuple, (ASTGameInference) keyword);
        } else if (keyword instanceof ASTGameInit) {
            handleRootKeyword(tuple, (ASTGameInit) keyword);
        } else {
            handleRootFact(tuple);
        }
    }

    private void handleRootKeyword(ASTGameTuple tuple, ASTGameInference keyword) {
        final ASTGameConstruct sentenceImplied = tuple.getElement(1);

        handleBodyConstruct(sentenceImplied);
        if (tuple.getElementList().size() <= 2) {
            println(".");
            return;
        }
        println(" :-");
        indent();

        printElementsSeparated(tuple.getElementList(), 2, ELEMENT_SEPARATOR, true, this::handleBodyConstruct);

        print(".");
        unindent();
    }

    private void handleRootKeyword(ASTGameTuple tuple, ASTGameInit keyword) {
        keyword.accept(getTraverser());
        // distinct between hidden and normal state
        ASTGameConstruct construct = tuple.getElement(1);
        handleDecideHidden(tuple, construct);
        print(".");
    }

    private void handleRootFact(ASTGameTuple tuple) {
        handleFact(tuple);
        print(".");
    }

    private void handleFact(ASTGameConstruct construct) {
        print(RULE_PREFIX);
        print("(");
        construct.accept(getTraverser());
        print(")");
    }

    private void handleBodyConstruct(ASTGameConstruct construct) {
        if (construct instanceof ASTGameTuple) {
            final ASTGameTuple tuple = (ASTGameTuple) construct;
            final ASTGameConstruct first = tuple.getElement(0);

            if (first instanceof ASTGameKeyword) {
                handleBodyKeyword(tuple, (ASTGameKeyword) first);
            } else {
                handleFact(tuple);
            }
        } else {
            handleFact(construct);
        }
    }

    private void handleBodyKeyword(ASTGameTuple tuple, ASTGameKeyword keyword) {
        if (keyword instanceof ASTGameTrue) {
            handleBodyKeyword(tuple, (ASTGameTrue) keyword);
        } else if (keyword instanceof ASTGameNot) {
            handleBodyKeyword(tuple, (ASTGameNot) keyword);
        } else if (keyword instanceof ASTGameNext) {
            handleBodyKeyword(tuple, (ASTGameNext) keyword);
        } else if (keyword instanceof ASTGameCount) {
            handleBodyKeyword(tuple, (ASTGameCount) keyword);
        } else {
            handleFact(tuple);
        }
    }

    private void handleBodyKeyword(ASTGameTuple tuple, ASTGameTrue keyword) {
        keyword.accept(getTraverser());

        // distinct between hidden and normal state
        ASTGameConstruct construct = tuple.getElement(1);
        handleDecideHidden(tuple, construct);
    }

    private void handleBodyKeyword(ASTGameTuple tuple, ASTGameNot keyword) {
        keyword.accept(getTraverser());

        print("(");
        // distinct between hidden and normal state
        ASTGameConstruct construct = tuple.getElement(1);
        handleBodyConstruct(construct);
        print(")");
    }

    private void handleBodyKeyword(ASTGameTuple tuple, ASTGameNext keyword) {
        keyword.accept(getTraverser());

        // distinct between hidden and normal state
        ASTGameConstruct construct = tuple.getElement(1);
        handleDecideHidden(tuple, construct);
    }

    private void handleBodyKeyword(ASTGameTuple tuple, ASTGameCount keyword) {
        keyword.accept(getTraverser());
        print("(");
        tuple.getElement(1).accept(getTraverser());
        println(", (");
        indent();
        printElementsSeparated(tuple.getElementList(), 2, ",", true, this::handleBodyConstruct);
        println(")");
        unindent();
        print(")");
    }

    private boolean isHidden(ASTGameConstruct construct) {
        return construct instanceof ASTGameTuple && ((ASTGameTuple) construct).getElement(0) instanceof ASTGameSees;
    }

    private void handleDecideHidden(ASTGameTuple tuple, ASTGameConstruct construct) {
        if (isHidden(construct)) {
            print("_hidden(");
            ((ASTGameTuple) construct).getElement(1).accept(getTraverser());
            print(ELEMENT_SEPARATOR);
            ((ASTGameTuple) construct).getElement(2).accept(getTraverser());
            print(")");
        } else {
            print("(");
            printElementsSeparated(tuple.getElementList(), 1, ELEMENT_SEPARATOR, false, c -> c.accept(getTraverser()));
            print(")");
        }
    }

    @Override
    public void handle(ASTGameTuple tuple) {
        print("[");
        printElementsSeparated(tuple.getElementList(), 0, ELEMENT_SEPARATOR, false, c -> c.accept(getTraverser()));
        print("]");
    }

    // VISIT

    @Override
    public void visit(ASTGameValue node) {
        final String value = VALUE_PREFIX + PREFIX_SEPARATOR + node.getValue();
        print(value);
    }

    @Override
    public void visit(ASTGameDigits node) {
        final ASTSignedNatLiteral literal = node.getNumber();
        final int number = literal.getValue();
        final String numberString;

        if (number < 0) {
            numberString = DIGITS_NEGATIVE_PREFIX + PREFIX_SEPARATOR + (number*-1);
        } else {
            numberString = DIGITS_POSITIVE_PREFIX + PREFIX_SEPARATOR + number;
        }

        print(numberString);
    }

    @Override
    public void visit(ASTGameToken node) {
        final String token = TOKEN_PREFIX + PREFIX_SEPARATOR + node.getToken();
        print(token);
    }

    @Override
    public void visit(ASTGameNext node) {
        print("gdl_next");
    }

    // KEYWORDS

    @Override
    public void visit(ASTGameTrue node) {
        print("gdl_state");
    }

    @Override
    public void visit(ASTGameInit node) {
        print("gdl_init");
    }

    @Override
    public void visit(ASTGameCount node) {
        print("gdl_count");
    }

    public void visit(ASTGameNot node) {
        print("gdl_not");
    }

    // STATIC KEYWORDS

    @Override
    public void visit(ASTGameDistinct node) {
        print("distinct");
    }

    @Override
    public void visit(ASTGameRole node) {
        print("role");
    }

    @Override
    public void visit(ASTGameLegal node) {
        print("legal");
    }

    public void visit(ASTGameDoes node) {
        print("does");
    }

    @Override
    public void visit(ASTGameGoal node) {
        print("goal");
    }

    @Override
    public void visit(ASTGameTerminal node) {
        print("terminal");
    }

    @Override
    public void visit(ASTGameAdd node) {
        print("add");
    }
    
    @Override
    public void visit(ASTGameSub node) {
        print("sub");
    }
    
    @Override
    public void visit(ASTGameMult node) {
        print("mult");
    }
    
    @Override
    public void visit(ASTGameDiv node) {
        print("div");
    }
    
    @Override
    public void visit(ASTGameSucc node) {
        print("succ");
    }
    
    @Override
    public void visit(ASTGameLess node) {
        print("less");
    }
    
    @Override
    public void visit(ASTGameGreater node) {
        print("greater");
    }
    
    @Override
    public void visit(ASTGameEqual node) {
        print("equal");
    }
    
    @Override
    public void visit(ASTGameNumber node) {
        print("number");
    }
    
    @Override
    public void visit(ASTGameMod node) {
        print("mod");
    }
    
    @Override
    public void visit(ASTGameTypeDef node) {
        print("type");
    }
    
    @Override
    public void visit(ASTGameTypeMapDef node) {
        print("typemap");
    }
    
    @Override
    public void visit(ASTGameTypeCombineDef node) {
        print("typecombine");
    }
    
    @Override
    public void visit(ASTGameNoop node) {
        print(VALUE_PREFIX + PREFIX_SEPARATOR + "noop");
    }
    
}
