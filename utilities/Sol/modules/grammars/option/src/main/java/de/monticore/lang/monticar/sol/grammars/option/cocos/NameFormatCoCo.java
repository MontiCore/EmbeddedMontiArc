/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.option.cocos;

import de.monticore.lang.monticar.sol.grammars.option._ast.ASTOption;
import de.monticore.lang.monticar.sol.grammars.option._ast.ASTOptionType;
import de.monticore.lang.monticar.sol.grammars.option._ast.ASTPropDeclaration;
import de.monticore.lang.monticar.sol.grammars.option._cocos.OptionASTOptionCoCo;
import de.monticore.lang.monticar.sol.grammars.option._cocos.OptionASTOptionTypeCoCo;
import de.monticore.lang.monticar.sol.grammars.option._cocos.OptionASTPropDeclarationCoCo;
import de.monticore.lang.monticar.sol.grammars.option._cocos.OptionCoCoChecker;

import java.util.regex.Pattern;

public class NameFormatCoCo extends CommonOptionCoCo
        implements OptionASTOptionCoCo, OptionASTOptionTypeCoCo, OptionASTPropDeclarationCoCo {
    protected final Pattern lCamelCase;
    protected final Pattern uCamelCase;
    protected final Pattern lCamelCaseStrict;

    public NameFormatCoCo() {
        super("OPT0009", "Name of %s '%s' should match '%s'.");

        this.uCamelCase = Pattern.compile("[[A-Z]_][[a-z][A-Z][0-9]_]*");
        this.lCamelCase = Pattern.compile("[[a-z]_][[a-z][A-Z][0-9]_]*");
        this.lCamelCaseStrict = Pattern.compile("[[a-z]][[a-z][A-Z][0-9]_]*");
    }

    @Override
    public void registerTo(OptionCoCoChecker checker) {
        checker.addCoCo((OptionASTOptionCoCo) this);
        checker.addCoCo((OptionASTOptionTypeCoCo) this);
        checker.addCoCo((OptionASTPropDeclarationCoCo) this);
    }

    @Override
    public void check(ASTOption node) {
        String name = node.getName();

        if (!this.lCamelCase.matcher(name).matches()) this.warn(node, "option", name, this.lCamelCase.pattern());
    }

    @Override
    public void check(ASTOptionType node) {
        String name = node.getName();

        if (!this.uCamelCase.matcher(name).matches()) this.warn(node, "option type", name, this.uCamelCase.pattern());
    }

    @Override
    public void check(ASTPropDeclaration node) {
        String name = node.getName();

        if (!this.lCamelCaseStrict.matcher(name).matches()) this.warn(node, "prop declaration", name, this.lCamelCaseStrict.pattern());
    }
}
