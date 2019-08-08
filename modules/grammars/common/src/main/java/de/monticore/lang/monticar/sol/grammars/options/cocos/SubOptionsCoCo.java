/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.grammars.options.cocos;

import com.google.inject.Inject;
import de.monticore.lang.monticar.sol.grammars.options._ast.ASTOption;
import de.monticore.lang.monticar.sol.grammars.options._cocos.OptionsASTOptionCoCo;
import de.monticore.lang.monticar.sol.grammars.options._cocos.OptionsCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.options._visitor.OptionsVisitor;
import de.monticore.lang.monticar.sol.grammars.options.cocos.types.OptionType;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;

/**
 * This context condition checks whether a declared option accepts sub-option declarations.
 */
public class SubOptionsCoCo implements OptionCoCo, OptionsASTOptionCoCo, OptionsVisitor {
    protected final Map<String, OptionType> types;

    protected ASTOption node;
    protected OptionType type;

    @Inject
    protected SubOptionsCoCo(Map<String, OptionType> types) {
        this.types = types;
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
        this.type = this.types.get(node.getType());

        this.traverse(node);
    }

    @Override
    public void visit(ASTOption peer) {
        String type = this.node.getType();

        if (!this.type.allowsSubOptions()) Log.warn(this.getErrorMessage(type), this.node.get_SourcePositionStart());
    }
}
