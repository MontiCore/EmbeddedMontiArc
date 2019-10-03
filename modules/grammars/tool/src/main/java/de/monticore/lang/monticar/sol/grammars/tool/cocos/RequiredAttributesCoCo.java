/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.tool.cocos;

import de.monticore.ast.ASTNode;
import de.monticore.lang.monticar.sol.grammars.tool._symboltable.AttributableSymbol;
import de.monticore.lang.monticar.sol.grammars.tool._symboltable.AttributeSymbol;
import de.monticore.lang.monticar.sol.grammars.tool.cocos.schema.Schema;
import de.monticore.symboltable.Symbol;
import de.se_rwth.commons.logging.Log;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

/**
 * This context condition checks whether all model elements have the attributes that they require.
 */
public class RequiredAttributesCoCo extends AbstractSchemaCoCo {
    public RequiredAttributesCoCo() throws IOException {
        super();
    }

    @Override
    public String getErrorCode() {
        return "TOOL0001";
    }

    @Override
    public String getErrorMessage(Object... parameters) {
        List<Object> parameterList = new ArrayList<>(Arrays.asList(parameters));

        parameterList.add(0, this.getErrorCode());
        return String.format("%s '%s' are required but are missing.", parameterList.toArray());
    }

    @Override
    protected void check(AttributableSymbol symbol, Schema schema) {
        Symbol realSymbol = (Symbol)symbol;

        realSymbol.getAstNode().ifPresent(node -> this.check(symbol, schema, node));
    }

    protected void check(AttributableSymbol symbol, Schema schema, ASTNode node) {
        List<Object> requiredAttributes = schema.getRequiredAttributes();
        Set<String> attributes = symbol.getAttributeSymbols().stream()
                .map(AttributeSymbol::getName)
                .collect(Collectors.toSet());

        requiredAttributes.removeAll(attributes);

        if (!requiredAttributes.isEmpty()) Log.warn(this.getErrorMessage(requiredAttributes), node.get_SourcePositionStart());
    }
}
