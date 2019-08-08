/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.grammars.options.cocos;

import com.google.inject.Inject;
import de.monticore.lang.monticar.sol.grammars.options._ast.ASTOptionProp;
import de.monticore.lang.monticar.sol.grammars.options._ast.ASTPair;
import de.monticore.lang.monticar.sol.grammars.options._cocos.OptionsASTOptionPropCoCo;
import de.monticore.lang.monticar.sol.grammars.options._cocos.OptionsCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.options._visitor.OptionsVisitor;
import de.monticore.lang.monticar.sol.grammars.options.cocos.types.props.Prop;
import de.monticore.mcliterals._ast.ASTBooleanLiteral;
import de.monticore.mcliterals._ast.ASTDoubleLiteral;
import de.monticore.mcliterals._ast.ASTStringLiteral;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;

/**
 * This context condition checks whether a prop accepts the given type.
 */
public class PropTypeCoCo implements OptionCoCo, OptionsASTOptionPropCoCo, OptionsVisitor {
    protected final Map<String, Prop> props;

    protected ASTPair node;
    protected Prop prop;

    @Inject
    protected PropTypeCoCo(Map<String, Prop> props) {
        this.props = props;
    }

    @Override
    public String getErrorCode() {
        return "OPT0002";
    }

    @Override
    public String getErrorMessage(Object... parameters) {
        List<Object> parameterList = new ArrayList<>(Arrays.asList(parameters));

        parameterList.add(0, this.getErrorCode());
        return String.format("%s '%s' does not accept a value of type '%s'.", parameterList.toArray());
    }

    @Override
    public void registerTo(OptionsCoCoChecker checker) {
        checker.addCoCo(this);
    }

    @Override
    public void check(ASTOptionProp node) {
        this.handle(node);
    }

    @Override
    public void visit(ASTPair node) {
        this.node = node;
        this.prop = this.props.get(node.getKey());
    }

    @Override
    public void visit(ASTStringLiteral node) {
        if (!this.prop.accepts(node)) this.log("string");
    }

    @Override
    public void visit(ASTDoubleLiteral node) {
        if (!this.prop.accepts(node)) this.log("double");
    }

    @Override
    public void visit(ASTBooleanLiteral node) {
        if (!this.prop.accepts(node)) this.log("boolean");
    }

    protected void log(String type) {
        Log.warn(this.getErrorMessage(this.prop.getIdentifier(), type), this.node.get_SourcePositionStart());
    }
}
