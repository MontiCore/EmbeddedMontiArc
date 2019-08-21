/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.lc.plugin.generator.ld;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.grammars.language._ast.ASTLanguageCompilationUnit;
import de.monticore.lang.monticar.sol.grammars.language._symboltable.LanguageSymbol;
import de.monticore.lang.monticar.sol.grammars.language._symboltable.TemplateDeclarationSymbol;
import de.monticore.lang.monticar.sol.grammars.language._symboltable.TemplateUndeclarationSymbol;
import de.monticore.lang.monticar.sol.grammars.language._visitor.LanguageVisitor;
import de.monticore.lang.monticar.sol.grammars.options._symboltable.OptionSymbol;
import de.monticore.lang.monticar.sol.grammars.options.visitor.OptionsSerializer;
import de.monticore.symboltable.CommonSymbol;
import org.json.JSONArray;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

@Singleton
public class LDExtractorImpl implements LDExtractor, LanguageVisitor {
    protected final OptionsSerializer serializer;

    @Inject
    protected LDExtractorImpl(OptionsSerializer serializer) {
        this.serializer = serializer;
    }

    @Override
    public List<TemplateDeclarationSymbol> getTemplateDeclarations(ASTLanguageCompilationUnit node) {
        return node.getLanguage()
                .getLanguageSymbolOpt()
                .map(LanguageSymbol::getLocalDeclarationSymbols)
                .orElse(new ArrayList<>());
    }

    @Override
    public List<TemplateUndeclarationSymbol> getTemplateUndeclarations(ASTLanguageCompilationUnit node) {
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
    public String getIdentifier(TemplateUndeclarationSymbol symbol) {
        return symbol.getMatchingDeclarationSymbol()
                .map(CommonSymbol::getFullName)
                .orElseThrow(() -> new RuntimeException("No matching declaration."));
    }

    @Override
    public String getPath(TemplateDeclarationSymbol symbol) {
        return symbol.getPath();
    }

    @Override
    public String getLabel(TemplateDeclarationSymbol symbol) {
        return symbol.getAttributeSymbols().stream()
                .filter(attribute -> attribute.getName().equals("label"))
                .findFirst()
                .map(attribute -> (String)attribute.getValue())
                .get();
    }

    @Override
    public JSONArray getElements(TemplateDeclarationSymbol symbol) {
        JSONArray result = new JSONArray();

        symbol.getOptionSymbols().stream()
                .map(OptionSymbol::getOptionNode)
                .filter(Optional::isPresent)
                .map(Optional::get)
                .forEach(node -> result.put(this.serializer.serialize(node)));

        return result;
    }
}
