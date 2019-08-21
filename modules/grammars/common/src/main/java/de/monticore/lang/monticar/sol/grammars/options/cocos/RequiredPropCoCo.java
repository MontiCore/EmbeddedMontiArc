/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.grammars.options.cocos;

import com.google.inject.Inject;
import de.monticore.lang.monticar.sol.grammars.options._ast.ASTOption;
import de.monticore.lang.monticar.sol.grammars.options._ast.ASTOptionProp;
import de.monticore.lang.monticar.sol.grammars.options._cocos.OptionsASTOptionCoCo;
import de.monticore.lang.monticar.sol.grammars.options._cocos.OptionsCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.options._visitor.OptionsVisitor;
import de.se_rwth.commons.logging.Log;

import java.util.*;

/**
 * This context condition checks whether all required props are present in the option declaration.
 */
public class RequiredPropCoCo implements OptionCoCo, OptionsASTOptionCoCo, OptionsVisitor {
    protected final ComponentTypeService service;
    protected final Set<String> props;

    protected ASTOption node;

    @Inject
    protected RequiredPropCoCo(ComponentTypeService service) {
        this.service = service;
        this.props = new HashSet<>();
    }

    @Override
    public String getErrorCode() {
        return "OPT0004";
    }

    @Override
    public String getErrorMessage(Object... parameters) {
        List<Object> parameterList = new ArrayList<>(Arrays.asList(parameters));

        parameterList.add(0, this.getErrorCode());
        return String.format("%s '%s' requires prop '%s'.", parameterList.toArray());
    }

    @Override
    public void registerTo(OptionsCoCoChecker checker) {
        checker.addCoCo(this);
    }

    @Override
    public void check(ASTOption node) {
        Set<String> requiredProps = this.service.getRequiredProps(node.getType());

        this.props.clear();
        this.handle(node);

        for (String prop : requiredProps) {
            if (!this.props.contains(prop)) Log.warn(this.getErrorMessage(node.getType(), prop), node.get_SourcePositionStart());
        }
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
        this.props.add(node.getName());
    }
}
