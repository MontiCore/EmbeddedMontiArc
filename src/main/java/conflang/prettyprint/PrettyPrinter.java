/* (c) https://github.com/MontiCore/monticore */
package conflang.prettyprint;

import conflang.ConfLangMill;
import conflang._ast.ASTConfLang;
import conflang._ast.ASTSimpleConfigurationEntry;
import conflang._visitor.ConfLangHandler;
import conflang._visitor.ConfLangTraverser;
import de.monticore.literals.mccommonliterals._ast.ASTNatLiteral;
import de.monticore.literals.mccommonliterals._ast.ASTSignedNatLiteral;
import de.monticore.prettyprint.IndentPrinter;

/**
 * Pretty prints configurations. Use {@link #handle(ASTConfLang)} to start a pretty
 * print and get the result by using {@link #getResult()}.
 */
public class PrettyPrinter implements ConfLangHandler {

    private final IndentPrinter printer;
    private ConfLangTraverser traverser;

    public PrettyPrinter() {
        this.printer = new IndentPrinter();
        this.traverser = ConfLangMill.traverser();
        traverser.setConfLangHandler(this);
    }

    /**
     * Gets the printed result.
     *
     * @return the result of the pretty print.
     */
    public String getResult() {
        return this.printer.getContent();
    }

    /**
     * Gets the printer.
     *
     * @return the printer of this.
     */
    public IndentPrinter getPrinter() {
        return this.printer;
    }

    @Override
    public void handle(ASTConfLang node) {
        getPrinter().println("configuration " + node.getName() + " {");
        getPrinter().indent();
        getTraverser().traverse(node);
        getPrinter().unindent();
        getPrinter().println("}");
    }

    @Override
    public void traverse(ASTConfLang node) {
        node.getConfigurationEntryList().forEach(s -> s.accept(getTraverser()));
    }

    @Override
    public void handle(ASTSimpleConfigurationEntry node) {
        getPrinter().print(node.getName());
        getPrinter().print(" = ");

        if (node.getValue() instanceof ASTSignedNatLiteral) {
            ASTSignedNatLiteral literal = (ASTSignedNatLiteral) node.getValue();
            getPrinter().print(literal.getValue());
        }
        getPrinter().println();
    }

    @Override
    public ConfLangTraverser getTraverser() {
        return this.traverser;
    }

    @Override
    public void setTraverser(ConfLangTraverser traverser) {
        this.traverser = traverser;
    }
}
