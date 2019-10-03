/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.tool.cocos;

import de.monticore.lang.monticar.sol.grammars.tool._ast.ASTResource;
import de.monticore.lang.monticar.sol.grammars.tool._ast.ASTResources;
import de.monticore.lang.monticar.sol.grammars.tool._ast.ASTTool;
import de.monticore.lang.monticar.sol.grammars.tool._cocos.ToolASTResourceCoCo;
import de.monticore.lang.monticar.sol.grammars.tool._cocos.ToolASTResourcesCoCo;
import de.monticore.lang.monticar.sol.grammars.tool._cocos.ToolASTToolCoCo;
import de.monticore.lang.monticar.sol.grammars.tool._cocos.ToolCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.tool._symboltable.AttributableSymbol;
import de.monticore.lang.monticar.sol.grammars.tool._symboltable.AttributeSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

/**
 * This context condition checks whether no attribute has been declared multiple times.
 */
public class UniqueAttributeCoCo implements ToolCoCo, ToolASTToolCoCo, ToolASTResourcesCoCo, ToolASTResourceCoCo {
    protected static final String MESSAGE = "Symbol for node could not be found.";

    @Override
    public String getErrorCode() {
        return "TOOL0003";
    }

    @Override
    public String getErrorMessage(Object... parameters) {
        List<Object> parameterList = new ArrayList<>(Arrays.asList(parameters));

        parameterList.add(0, this.getErrorCode());
        return String.format("%s An attribute with name '%s' already exists.", parameterList.toArray());
    }

    @Override
    public void registerTo(ToolCoCoChecker checker) {
        checker.addCoCo((ToolASTToolCoCo) this);
        checker.addCoCo((ToolASTResourcesCoCo) this);
        checker.addCoCo((ToolASTResourceCoCo) this);
    }

    @Override
    public void check(ASTTool node) {
        if (node.isPresentToolSymbol()) this.check(node.getToolSymbol());
        else Log.warn(MESSAGE, node.get_SourcePositionStart());
    }

    @Override
    public void check(ASTResources node) {
        if (node.isPresentResourcesSymbol()) this.check(node.getResourcesSymbol());
        else Log.warn(MESSAGE, node.get_SourcePositionStart());
    }

    @Override
    public void check(ASTResource node) {
        if (node.isPresentResourceSymbol()) this.check(node.getResourceSymbol());
        else Log.warn(MESSAGE, node.get_SourcePositionStart());
    }

    protected void check(AttributableSymbol symbol) {
        List<String> identifiers = symbol.getAttributeSymbols().stream()
                .map(AttributeSymbol::getName)
                .collect(Collectors.toList());

        symbol.getAttributeSymbols().forEach(s -> this.check(s, identifiers));
    }

    protected void check(AttributeSymbol symbol, List<String> identifiers) {
        String identifier = symbol.getName();

        identifiers.remove(identifier);

        symbol.getAstNode().ifPresent(node -> {
            if (identifiers.contains(identifier)) Log.warn(this.getErrorMessage(identifier), node.get_SourcePositionStart());
        });
    }
}
