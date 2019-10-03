/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.tool.cocos;

import de.monticore.lang.monticar.sol.grammars.tool._ast.ASTResource;
import de.monticore.lang.monticar.sol.grammars.tool._cocos.ToolASTResourceCoCo;
import de.monticore.lang.monticar.sol.grammars.tool._cocos.ToolCoCoChecker;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * This context condition checks whether the format of a resource's name is valid.
 */
public class ResourceNameCoCo implements ToolCoCo, ToolASTResourceCoCo {
    protected final Pattern pattern;

    public ResourceNameCoCo() {
        this.pattern = Pattern.compile("[_A-Z]+");
    }

    @Override
    public String getErrorCode() {
        return "TOOL0002";
    }

    @Override
    public String getErrorMessage(Object... parameters) {
        List<Object> parameterList = new ArrayList<>(Arrays.asList(parameters));

        parameterList.add(0, this.getErrorCode());
        return String.format("%s '%s' should be all CAPITAL_CASE.", parameterList.toArray());
    }

    @Override
    public void registerTo(ToolCoCoChecker checker) {
        checker.addCoCo(this);
    }

    @Override
    public void check(ASTResource node) {
        String identifier = node.getName();
        Matcher matcher = this.pattern.matcher(identifier);

        if (!matcher.matches()) Log.warn(this.getErrorMessage(identifier), node.get_SourcePositionStart());
    }
}
