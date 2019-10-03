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
import de.monticore.lang.monticar.sol.grammars.tool._symboltable.ToolSymbol;
import de.monticore.lang.monticar.sol.grammars.tool.cocos.schema.Schema;
import de.se_rwth.commons.logging.Log;

import java.io.IOException;

/**
 * This class acts as common ground for all schema related context conditions.
 */
public abstract class AbstractSchemaCoCo implements ToolCoCo, ToolASTToolCoCo, ToolASTResourcesCoCo, ToolASTResourceCoCo {
    protected static final String MESSAGE = "Symbol for node could not be found.";

    protected final Schema resourceSchema;
    protected final Schema resourcesSchema;
    protected final Schema toolSchema;
    protected final Schema virtualToolSchema;

    protected AbstractSchemaCoCo() throws IOException {
        this.resourceSchema = new Schema("resource.schema.json");
        this.resourcesSchema = new Schema("resources.schema.json");
        this.toolSchema = new Schema("tool.schema.json");
        this.virtualToolSchema = new Schema("virtual-tool.schema.json");
    }

    @Override
    public void registerTo(ToolCoCoChecker checker) {
        checker.addCoCo((ToolASTToolCoCo) this);
        checker.addCoCo((ToolASTResourcesCoCo) this);
        checker.addCoCo((ToolASTResourceCoCo) this);
    }

    @Override
    public void check(ASTTool node) {
        if (node.isPresentToolSymbol()) {
            ToolSymbol symbol = node.getToolSymbol();

            if (symbol.isVirtual()) this.check(symbol, this.virtualToolSchema);
            else this.check(symbol, this.toolSchema);
        } else {
            Log.warn(MESSAGE, node.get_SourcePositionStart());
        }
    }

    @Override
    public void check(ASTResource node) {
        if (node.isPresentResourceSymbol()) this.check(node.getResourceSymbol(), this.resourceSchema);
        else Log.warn(MESSAGE, node.get_SourcePositionStart());
    }

    @Override
    public void check(ASTResources node) {
        if (node.isPresentResourcesSymbol()) this.check(node.getResourcesSymbol(), this.resourcesSchema);
        else Log.warn(MESSAGE, node.get_SourcePositionStart());
    }

    protected abstract void check(AttributableSymbol symbol, Schema schema);
}
