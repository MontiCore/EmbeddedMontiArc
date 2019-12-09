/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.ide.cocos;

import de.monticore.ast.ASTNode;
import de.monticore.lang.monticar.sol.grammars.ide._ast.*;
import de.monticore.lang.monticar.sol.grammars.ide._cocos.*;

import java.util.regex.Pattern;

public class NameFormatCoCo extends CommonIDECoCo
        implements IDEASTIDECoCo, IDEASTConfigurationCoCo, IDEASTConfigurationTypeCoCo, IDEASTModuleCoCo, IDEASTModuleTypeCoCo {
    protected final Pattern upperCase;
    protected final Pattern lowerCase;

    public NameFormatCoCo() {
        super("IDE0000", "Name of %s '%s' should match '%s'.");

        this.upperCase = Pattern.compile("[[A-Z]_][[a-z][A-Z][0-9]_]*");
        this.lowerCase = Pattern.compile("[[a-z]_][[a-z][A-Z][0-9]_]*");
    }

    @Override
    public void registerTo(IDECoCoChecker checker) {
        checker.addCoCo((IDEASTIDECoCo) this);
        checker.addCoCo((IDEASTConfigurationTypeCoCo) this);
        checker.addCoCo((IDEASTConfigurationCoCo) this);
        checker.addCoCo((IDEASTModuleTypeCoCo) this);
        checker.addCoCo((IDEASTModuleCoCo) this);
    }

    @Override
    public void check(ASTConfiguration node) {
        this.checkLowerCase(node, "configuration", node.getName());
    }

    @Override
    public void check(ASTModule node) {
        this.checkLowerCase(node, "module", node.getName());
    }

    @Override
    public void check(ASTIDE node) {
        this.checkUpperCase(node, "ide", node.getName());
    }

    @Override
    public void check(ASTConfigurationType node) {
        this.checkUpperCase(node, "configuration type", node.getName());
    }

    @Override
    public void check(ASTModuleType node) {
        this.checkUpperCase(node, "module type", node.getName());
    }

    protected void checkUpperCase(ASTNode node, String type, String name) {
        if (!this.upperCase.matcher(name).matches()) this.warn(node, type, name, this.upperCase.pattern());
    }

    protected void checkLowerCase(ASTNode node, String type, String name) {
        if (!this.lowerCase.matcher(name).matches()) this.warn(node, type, name, this.lowerCase.pattern());
    }
}
