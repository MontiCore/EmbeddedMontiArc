/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.tool.cocos;

import de.monticore.ast.ASTNode;
import de.monticore.lang.monticar.sol.grammars.tool._symboltable.AttributableSymbol;
import de.monticore.lang.monticar.sol.grammars.tool._symboltable.AttributeSymbol;
import de.monticore.lang.monticar.sol.grammars.tool.cocos.schema.Schema;
import de.se_rwth.commons.SourcePosition;
import de.se_rwth.commons.logging.Log;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * This context condition checks whether all model elements have only supported attributes.
 */
public class ValidAttributeCoCo extends AbstractSchemaCoCo {
    public ValidAttributeCoCo() throws IOException {
        super();
    }

    @Override
    public String getErrorCode() {
        return "TOOL0006";
    }

    @Override
    public String getErrorMessage(Object... parameters) {
        List<Object> parameterList = new ArrayList<>(Arrays.asList(parameters));

        parameterList.add(0, this.getErrorCode());
        return String.format("%s '%s' is not a valid attribute.", parameterList.toArray());
    }

    @Override
    protected void check(AttributableSymbol symbol, Schema schema) {
        symbol.getAttributeSymbols().forEach(s -> this.check(s, schema));
    }

    protected void check(AttributeSymbol symbol, Schema schema) {
        String identifier = symbol.getName();
        SourcePosition defaultSourcePosition = SourcePosition.getDefaultSourcePosition();
        SourcePosition sourcePosition = symbol.getAttributeNode().map(ASTNode::get_SourcePositionStart).orElse(defaultSourcePosition);

        if(!schema.hasAttribute(identifier)) Log.warn(this.getErrorMessage(identifier), sourcePosition);
    }
}
