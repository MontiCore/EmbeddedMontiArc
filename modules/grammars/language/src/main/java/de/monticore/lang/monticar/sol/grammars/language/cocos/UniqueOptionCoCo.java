/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.grammars.language.cocos;

import de.monticore.lang.monticar.sol.grammars.language._ast.ASTTemplateDeclaration;
import de.monticore.lang.monticar.sol.grammars.language._cocos.LanguageASTTemplateDeclarationCoCo;
import de.monticore.lang.monticar.sol.grammars.language._cocos.LanguageCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.language._visitor.LanguageVisitor;

/**
 * This context condition checks whether all options in a template declaration have unique identifiers.
 */
public class UniqueOptionCoCo extends de.monticore.lang.monticar.sol.grammars.options.cocos.UniqueOptionCoCo
        implements LanguageCoCo, LanguageASTTemplateDeclarationCoCo, LanguageVisitor {
    @Override
    public void registerTo(LanguageCoCoChecker checker) {
        checker.addCoCo((LanguageASTTemplateDeclarationCoCo)this);
    }

    @Override
    public void check(ASTTemplateDeclaration node) {
        this.identifiers.clear();
        this.traverse(node);
    }
}
