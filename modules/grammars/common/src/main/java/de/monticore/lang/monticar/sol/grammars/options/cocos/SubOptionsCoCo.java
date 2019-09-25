/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.options.cocos;

import com.google.inject.Inject;
import de.monticore.lang.monticar.sol.grammars.options._ast.ASTOption;
import de.monticore.lang.monticar.sol.grammars.options._cocos.OptionsASTOptionCoCo;
import de.monticore.lang.monticar.sol.grammars.options._cocos.OptionsCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.options._visitor.OptionsVisitor;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * This context condition checks whether a declared option accepts sub-option declarations.
 */
public class SubOptionsCoCo implements OptionCoCo, OptionsASTOptionCoCo, OptionsVisitor {
    protected final ComponentTypeService service;

    protected ASTOption node;
    protected String type;

    @Inject
    protected SubOptionsCoCo(ComponentTypeService service) {
        this.service = service;
    }

    @Override
    public String getErrorCode() {
        return "OPT0001";
    }

    @Override
    public String getErrorMessage(Object... parameters) {
        List<Object> parameterList = new ArrayList<>(Arrays.asList(parameters));

        parameterList.add(0, this.getErrorCode());
        return String.format("%s Sub-Options are not allowed for type '%s'.", parameterList.toArray());
    }

    @Override
    public void registerTo(OptionsCoCoChecker checker) {
        checker.addCoCo(this);
    }

    @Override
    public void check(ASTOption node) {
        this.node = node;
        this.type = node.getType();

        this.traverse(node);
    }

    @Override
    public void visit(ASTOption peer) {
        if (!this.service.supportsOptions(this.type)) Log.warn(this.getErrorMessage(this.type), this.node.get_SourcePositionStart());
    }
}
