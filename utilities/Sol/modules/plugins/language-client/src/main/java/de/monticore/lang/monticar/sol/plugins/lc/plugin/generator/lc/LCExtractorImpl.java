/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.lc.plugin.generator.lc;

import com.google.gson.Gson;
import com.google.gson.JsonArray;
import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.grammars.language._ast.ASTLanguageCompilationUnit;
import de.monticore.lang.monticar.sol.grammars.language._symboltable.LanguageSymbol;
import de.monticore.lang.monticar.sol.grammars.language._symboltable.TemplateDeclarationSymbol;
import de.monticore.lang.monticar.sol.grammars.language._symboltable.TemplateExclusionSymbol;
import de.monticore.lang.monticar.sol.grammars.language._visitor.LanguageVisitor;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.OptionSymbol;
import de.monticore.symboltable.CommonSymbol;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

@Singleton
public class LCExtractorImpl implements LCExtractor, LanguageVisitor {
    protected final Gson gson;

    @Inject
    protected LCExtractorImpl(Gson gson) {
        this.gson = gson;
    }

    @Override
    public List<TemplateDeclarationSymbol> getTemplateDeclarations(ASTLanguageCompilationUnit node) {
        return node.getLanguage()
                .getLanguageSymbolOpt()
                .map(LanguageSymbol::getLocalDeclarationSymbols)
                .orElse(new ArrayList<>());
    }

    @Override
    public List<TemplateExclusionSymbol> getTemplateUndeclarations(ASTLanguageCompilationUnit node) {
        return node.getLanguage()
                .getLanguageSymbolOpt()
                .map(LanguageSymbol::getLocalUndeclarationSymbols)
                .orElse(new ArrayList<>());
    }

    @Override
    public String getIdentifier(TemplateDeclarationSymbol symbol) {
        return symbol.getFullName();
    }

    @Override
    public String getIdentifier(TemplateExclusionSymbol symbol) {
        return symbol.getMatchingDeclarationSymbol()
                .map(CommonSymbol::getFullName)
                .orElseThrow(() -> new RuntimeException("No matching declaration."));
    }

    @Override
    public String getPath(TemplateDeclarationSymbol symbol) {
        return symbol.getPath().orElse(".");
    }

    @Override
    public String getLabel(TemplateDeclarationSymbol symbol) {
        return symbol.getLabel().orElse("");
    }
}
