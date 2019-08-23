/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.sol.grammars.options.cocos;

import com.google.inject.Inject;
import de.monticore.lang.monticar.sol.grammars.options._ast.ASTOption;
import de.monticore.lang.monticar.sol.grammars.options._ast.ASTOptionProp;
import de.monticore.lang.monticar.sol.grammars.options._cocos.OptionsASTOptionCoCo;
import de.monticore.lang.monticar.sol.grammars.options._cocos.OptionsCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.options._visitor.OptionsVisitor;
import de.monticore.mcliterals._ast.ASTBooleanLiteral;
import de.monticore.mcliterals._ast.ASTDoubleLiteral;
import de.monticore.mcliterals._ast.ASTStringLiteral;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * This context condition checks whether a prop accepts the given type.
 */
public class PropTypeCoCo implements OptionCoCo, OptionsASTOptionCoCo, OptionsVisitor {
    protected final ComponentTypeService service;

    protected ASTOptionProp node;
    protected String componentType;
    protected String prop;

    @Inject
    public PropTypeCoCo(ComponentTypeService service) {
        this.service = service;
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
    public void check(ASTOption node) {
        this.handle(node);
    }

    @Override
    public void visit(ASTOption node) {
        this.componentType = node.getType();
    }

    @Override
    public void visit(ASTOptionProp node) {
        this.node = node;
        this.prop = node.getName();
    }

    @Override
    public void visit(ASTStringLiteral node) {
        this.checkPropSupportsType("string");
    }

    @Override
    public void visit(ASTDoubleLiteral node) {
        this.checkPropSupportsType("double");
    }

    @Override
    public void visit(ASTBooleanLiteral node) {
        this.checkPropSupportsType("boolean");
    }

    protected void checkPropSupportsType(String type) {
        if (!this.service.propSupportsType(this.componentType, this.prop, type)) this.log(type);
    }

    protected void log(String type) {
        Log.warn(this.getErrorMessage(this.prop, type), this.node.get_SourcePositionStart());
    }
}
