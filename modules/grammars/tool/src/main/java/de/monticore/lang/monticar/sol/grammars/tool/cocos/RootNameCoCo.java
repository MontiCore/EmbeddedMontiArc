/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.tool.cocos;

import de.monticore.lang.monticar.sol.grammars.tool._ast.ASTResources;
import de.monticore.lang.monticar.sol.grammars.tool._ast.ASTTool;
import de.monticore.lang.monticar.sol.grammars.tool._cocos.ToolASTResourcesCoCo;
import de.monticore.lang.monticar.sol.grammars.tool._cocos.ToolASTToolCoCo;
import de.monticore.lang.monticar.sol.grammars.tool._cocos.ToolCoCoChecker;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * This context condition checks whether the name of a root model element (tool, virtual tool, resources) is valid.
 */
public class RootNameCoCo implements ToolCoCo, ToolASTToolCoCo, ToolASTResourcesCoCo {
    protected final Pattern pattern;

    public RootNameCoCo() {
        this.pattern = Pattern.compile("[A-Z][A-Za-z_]*");
    }

    @Override
    public String getErrorCode() {
        return "TOOL0005";
    }

    @Override
    public String getErrorMessage(Object... parameters) {
        List<Object> parameterList = new ArrayList<>(Arrays.asList(parameters));

        parameterList.add(0, this.getErrorCode());
        return String.format("%s '%s' should start with a capital letter.", parameterList.toArray());
    }

    @Override
    public void registerTo(ToolCoCoChecker checker) {
        checker.addCoCo((ToolASTToolCoCo) this);
        checker.addCoCo((ToolASTResourcesCoCo) this);
    }

    @Override
    public void check(ASTTool node) {
        String identifier = node.getName();
        Matcher matcher = this.pattern.matcher(identifier);

        if (!matcher.matches()) Log.warn(this.getErrorMessage(identifier), node.get_SourcePositionStart());
    }

    @Override
    public void check(ASTResources node) {
        String identifier = node.getName();
        Matcher matcher = this.pattern.matcher(identifier);

        if (!matcher.matches()) Log.warn(this.getErrorMessage(identifier), node.get_SourcePositionStart());
    }
}
