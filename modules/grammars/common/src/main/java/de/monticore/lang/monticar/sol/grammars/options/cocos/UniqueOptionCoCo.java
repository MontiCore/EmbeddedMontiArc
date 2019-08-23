/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.sol.grammars.options.cocos;

import de.monticore.lang.monticar.sol.grammars.options._ast.ASTOption;
import de.monticore.lang.monticar.sol.grammars.options._cocos.OptionsASTOptionCoCo;
import de.monticore.lang.monticar.sol.grammars.options._cocos.OptionsCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.options._visitor.OptionsVisitor;
import de.se_rwth.commons.logging.Log;

import java.util.*;

/**
 * This context condition checks whether the identifier used for a given option is unique in its scope.
 */
public class UniqueOptionCoCo implements OptionCoCo, OptionsASTOptionCoCo, OptionsVisitor {
    protected final Set<String> identifiers;

    protected UniqueOptionCoCo() {
        this.identifiers = new HashSet<>();
    }

    @Override
    public String getErrorCode() {
        return "OPT0005";
    }

    @Override
    public String getErrorMessage(Object... parameters) {
        List<Object> parameterList = new ArrayList<>(Arrays.asList(parameters));

        parameterList.add(0, this.getErrorCode());
        return String.format("%s There is already an option for '%s'.", parameterList.toArray());
    }

    @Override
    public void registerTo(OptionsCoCoChecker checker) {
        checker.addCoCo(this);
    }

    @Override
    public void check(ASTOption node) {
        this.identifiers.clear();
        this.traverse(node);
    }

    @Override
    public void handle(ASTOption node) {
        this.visit(node);
        this.endVisit(node);
    }

    @Override
    public void visit(ASTOption node) {
        String identifier = node.getName();

        if (this.identifiers.contains(identifier)) Log.warn(this.getErrorMessage(identifier), node.get_SourcePositionStart());
        else this.identifiers.add(identifier);
    }
}
