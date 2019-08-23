/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.sol.grammars.options.cocos;

import de.monticore.lang.monticar.sol.grammars.options._ast.ASTOption;
import de.monticore.lang.monticar.sol.grammars.options._ast.ASTOptionProp;
import de.monticore.lang.monticar.sol.grammars.options._cocos.OptionsASTOptionCoCo;
import de.monticore.lang.monticar.sol.grammars.options._cocos.OptionsCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.options._visitor.OptionsVisitor;
import de.se_rwth.commons.logging.Log;

import java.util.*;

/**
 * This context condition checks whether a prop appears more than a single time in an option declaration.
 */
public class NoDuplicatePropCoCo implements OptionCoCo, OptionsASTOptionCoCo, OptionsVisitor {
    protected final Set<String> props;

    protected ASTOption node;

    protected NoDuplicatePropCoCo() {
        this.props = new HashSet<>();
    }

    @Override
    public String getErrorCode() {
        return "OPT0006";
    }

    @Override
    public String getErrorMessage(Object... parameters) {
        List<Object> parameterList = new ArrayList<>(Arrays.asList(parameters));

        parameterList.add(0, this.getErrorCode());
        return String.format("%s '%s' is already defined in the same option.", parameterList.toArray());
    }

    @Override
    public void registerTo(OptionsCoCoChecker checker) {
        checker.addCoCo(this);
    }

    @Override
    public void check(ASTOption node) {
        this.props.clear();
        this.handle(node);
    }

    @Override
    public void handle(ASTOption node) {
        if (this.node == null) {
            this.visit(node);
            this.traverse(node);
            this.endVisit(node);
        }
    }

    @Override
    public void visit(ASTOption node) {
        this.node = node;
    }

    @Override
    public void endVisit(ASTOption node) {
        this.node = null;
    }

    @Override
    public void visit(ASTOptionProp node) {
        String prop = node.getName();

        if (this.props.contains(prop)) Log.warn(this.getErrorMessage(prop), node.get_SourcePositionStart());
        else this.props.add(prop);
    }
}
