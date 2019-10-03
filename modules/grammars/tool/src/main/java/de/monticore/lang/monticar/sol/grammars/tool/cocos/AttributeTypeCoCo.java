/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.tool.cocos;

import de.monticore.ast.ASTNode;
import de.monticore.lang.monticar.sol.grammars.tool._symboltable.AttributableSymbol;
import de.monticore.lang.monticar.sol.grammars.tool._symboltable.AttributeSymbol;
import de.monticore.lang.monticar.sol.grammars.tool._symboltable.attributes.LiteralAttributeSymbol;
import de.monticore.lang.monticar.sol.grammars.tool._visitor.ToolVisitor;
import de.monticore.lang.monticar.sol.grammars.tool.cocos.schema.Schema;
import de.monticore.mcliterals._ast.ASTBooleanLiteral;
import de.monticore.mcliterals._ast.ASTNumericLiteral;
import de.monticore.mcliterals._ast.ASTStringLiteral;
import de.se_rwth.commons.SourcePosition;
import de.se_rwth.commons.logging.Log;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

/**
 * This context condition checks whether all attributes are of the correct type.
 */
public class AttributeTypeCoCo extends AbstractSchemaCoCo implements ToolVisitor {
    protected static final String UNKNOWN_TYPE = "unknown";

    protected String literalType;

    public AttributeTypeCoCo() throws IOException {
        super();
    }

    @Override
    public String getErrorCode() {
        return "TOOL0000";
    }

    @Override
    public String getErrorMessage(Object... parameters) {
        List<Object> parameterList = new ArrayList<>(Arrays.asList(parameters));

        parameterList.add(0, this.getErrorCode());
        return String.format("%s '%s' is not of the right type '%s'.", parameterList.toArray());
    }

    @Override
    protected void check(AttributableSymbol symbol, Schema schema) {
        symbol.getAttributeSymbols().forEach(s -> this.check(s, schema));
    }

    protected void check(AttributeSymbol symbol, Schema schema) {
        String identifier = symbol.getName();
        String actualType = this.getTypeOfAttribute(symbol).orElse(UNKNOWN_TYPE);
        String expectedType = schema.hasAttribute(identifier) ? schema.getTypeOfAttribute(identifier) : UNKNOWN_TYPE;
        SourcePosition defaultSourcePosition = SourcePosition.getDefaultSourcePosition();
        SourcePosition sourcePosition = symbol.getAttributeNode().map(ASTNode::get_SourcePositionStart).orElse(defaultSourcePosition);

        if (!expectedType.equals(actualType)) Log.warn(this.getErrorMessage(identifier, expectedType), sourcePosition);
    }

    protected Optional<String> getTypeOfAttribute(AttributeSymbol symbol) {
        if (symbol.isAliasAttribute()) return Optional.of("alias");
        else if (symbol.isEnvironmentAttribute()) return Optional.of("name");
        else if (symbol.isLiteralAttribute()) return this.getTypeOfLiteralAttribute((LiteralAttributeSymbol) symbol);
        return Optional.of(UNKNOWN_TYPE);
    }

    protected Optional<String> getTypeOfLiteralAttribute(LiteralAttributeSymbol symbol) {
        this.literalType = null;

        symbol.getAttributeNode().ifPresent(this::handle);
        return Optional.ofNullable(this.literalType);
    }

    @Override
    public void visit(ASTStringLiteral node) {
        this.literalType = "string";
    }

    @Override
    public void visit(ASTBooleanLiteral node) {
        this.literalType = "boolean";
    }

    @Override
    public void visit(ASTNumericLiteral node) {
        this.literalType = "number";
    }
}
