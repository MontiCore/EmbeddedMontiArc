/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.language.cocos;

import de.monticore.lang.monticar.sol.grammars.language._ast.ASTTemplateExclusion;
import de.monticore.lang.monticar.sol.grammars.language._cocos.LanguageASTTemplateExclusionCoCo;
import de.monticore.lang.monticar.sol.grammars.language._cocos.LanguageCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.language._symboltable.TemplateExclusionSymbol;

/**
 * A context condition which checks whether an undeclaration refers to an existing declaration.
 */
public class ExcludeExistingCoCo extends CommonLanguageCoCo implements LanguageASTTemplateExclusionCoCo {
    public ExcludeExistingCoCo() {
        super("LANG0000", "Template '%s' must have been declared beforehand.");
    }

    @Override
    public void registerTo(LanguageCoCoChecker checker) {
        checker.addCoCo(this);
    }

    @Override
    public void check(ASTTemplateExclusion node) {
        node.getTemplateExclusionSymbolOpt().ifPresent(this::check);
    }

    protected void check(TemplateExclusionSymbol exclusion) {
        if (!exclusion.getMatchingDeclarationSymbol().isPresent()) this.error(exclusion, exclusion.getName());
    }
}
