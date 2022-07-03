package de.monticore.lang.gdl.visitors;

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

    private static final String VALUE_TYPE_PREFIX = "type";

    private static final String PREFIX_SEPARATOR = "_";
    private static final String ELEMENT_SEPARATOR = ", ";

    private GDLTraverser traverser;

    public TypeTemplatePrinter() {
        this.traverser = GDLMill.traverser();
        this.traverser.add4GDL(this);
        this.traverser.add4MCCommonLiterals(this);
        this.traverser.setGDLHandler(this);
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

    @Override
    public void handle(ASTGame game) {
        printElementsSeparated(game.getTuplesList(), 0, "", true, this::handleRoot);
    }

    private void handleRoot(ASTGameTuple tuple) {
        final ASTGameConstruct first = tuple.getElement(0);

        if (first instanceof ASTGameInference) {
            handleRootKeyword(tuple, (ASTGameInference) first);
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
        print(TEMPLATE_PREFIX_ACTION);
        print("(");
        printElementsSeparated(tuple.getElementList(), 1, ELEMENT_SEPARATOR, false, this::handle);
        println(").");
    }

    public void handleDecideHidden(ASTGameConstruct construct) {
        print(TEMPLATE_PREFIX_STATE);
        print("(");
        if (construct instanceof ASTGameTuple) {
            ASTGameTuple tuple = (ASTGameTuple) construct;

            if (tuple.getElement(0) instanceof ASTGameSees) {
                printElementsSeparated(tuple.getElementList(), 1, ELEMENT_SEPARATOR, false, this::handle);
            }
        } else {
            handle(construct);
        }
        println(").");
    }
    
    @Override
    public void handle(ASTGameTuple tuple) {
        if (tuple.isPresentType()) {
            handleConstantType(tuple.getType());
        } else {
            print("[");
            printElementsSeparated(tuple.getElementList(), 0, ELEMENT_SEPARATOR, false, this::handle);
            print("]");
        }
    }

    @Override
    public void handle(ASTGameValue node) {
        handleConstantType(node.getType());
    }

    @Override
    public void handle(ASTGameDigits node) {
        handleConstantType(node.getType());
    }

    @Override
    public void handle(ASTGameToken node) {
        node.getType().accept(getTraverser());
    }

    public void handleConstantType(ASTGameType type) {
        print("[constant");
        print(ELEMENT_SEPARATOR);
        type.accept(getTraverser());
        print("]");
    }

    // TYPES

    public void visit(ASTGameRangeType node) {
        print("[range");
        print(ELEMENT_SEPARATOR);
        print(node.getStart().getValue());
        print(ELEMENT_SEPARATOR);
        print(node.getEnd().getValue());
        print("]");
    }

    public void visit(ASTGameValueType node) {
        print(VALUE_TYPE_PREFIX);
        print(PREFIX_SEPARATOR);
        print(node.getType());
    }
    
}
