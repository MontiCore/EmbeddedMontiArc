/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.language._symboltable;

import de.monticore.lang.monticar.sol.grammars.common._ast.CommonLiterals;
import de.monticore.lang.monticar.sol.grammars.common._symboltable.SymbolWithProjectPath;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

public class LanguageSymbol extends LanguageSymbolTOP implements SymbolWithProjectPath {
    protected final List<LanguageSymbolReference> parents;
    protected final List<TemplateDeclarationSymbolReference> declarations;
    protected final List<TemplateExclusionSymbolReference> undeclarations;
    protected final List<String> keywords;
    protected final List<String> includedKeywords;
    protected final List<String> excludedKeywords;

    protected String serverPath;
    protected CommonLiterals origin;
    protected String extension;

    public LanguageSymbol(String name) {
        super(name);

        this.parents = new ArrayList<>();
        this.declarations = new ArrayList<>();
        this.undeclarations = new ArrayList<>();
        this.keywords = new ArrayList<>();
        this.includedKeywords = new ArrayList<>();
        this.excludedKeywords = new ArrayList<>();
    }

    public void addParent(LanguageSymbolReference parent) {
        this.parents.add(parent);
    }

    public List<LanguageSymbolReference> getParents() {
        return Collections.unmodifiableList(this.parents);
    }

    public List<LanguageSymbol> getParentSymbols() {
        return this.getParents().stream()
                .filter(LanguageSymbolReference::existsReferencedSymbol)
                .map(LanguageSymbolReference::getReferencedSymbol)
                .collect(Collectors.toList());
    }

    public void addLocalDeclaration(TemplateDeclarationSymbolReference declaration) {
        this.declarations.add(declaration);
    }

    public List<TemplateDeclarationSymbolReference> getLocalDeclarations() {
        return Collections.unmodifiableList(this.declarations);
    }

    public List<TemplateDeclarationSymbol> getLocalDeclarationSymbols() {
        return this.getLocalDeclarations().stream()
                .filter(TemplateDeclarationSymbolReference::existsReferencedSymbol)
                .map(TemplateDeclarationSymbolReference::getReferencedSymbol)
                .collect(Collectors.toList());
    }

    public List<TemplateDeclarationSymbol> getAllDeclarationSymbols() {
        List<TemplateDeclarationSymbol> symbols = this.getParentSymbols().stream()
                .flatMap(parent -> parent.getAllDeclarationSymbols().stream())
                .collect(Collectors.toList());

        symbols.addAll(this.getLocalDeclarationSymbols());

        return symbols;
    }

    public void addLocalUndeclaration(TemplateExclusionSymbolReference undeclaration) {
        this.undeclarations.add(undeclaration);
    }

    public List<TemplateExclusionSymbolReference> getLocalUndeclarations() {
        return Collections.unmodifiableList(this.undeclarations);
    }

    public List<TemplateExclusionSymbol> getLocalUndeclarationSymbols() {
        return this.getLocalUndeclarations().stream()
                .filter(TemplateExclusionSymbolReference::existsReferencedSymbol)
                .map(TemplateExclusionSymbolReference::getReferencedSymbol)
                .collect(Collectors.toList());
    }

    public List<TemplateExclusionSymbol> getAllUndeclarationSymbols() {
        List<TemplateExclusionSymbol> symbols = this.getParentSymbols().stream()
                .flatMap(parent -> parent.getAllUndeclarationSymbols().stream())
                .collect(Collectors.toList());

        symbols.addAll(this.getLocalUndeclarationSymbols());

        return symbols;
    }

    @Override
    public void setPath(String serverPath) {
        this.serverPath = serverPath;
    }

    @Override
    public Optional<String> getPath() {
        return Optional.ofNullable(this.serverPath);
    }

    @Override
    public void setOrigin(CommonLiterals origin) {
        this.origin = origin;
    }

    @Override
    public Optional<CommonLiterals> getOrigin() {
        return Optional.ofNullable(this.origin);
    }

    public Optional<String> getServerPath() {
        return this.getPath();
    }

    public void setExtension(String extension) {
        this.extension = extension;
    }

    public Optional<String> getExtension() {
        return Optional.ofNullable(this.extension)
                .map(extension -> extension.startsWith(".") ? extension : ("." + extension));
    }

    public void includeKeywords(List<String> keywords) {
        this.includedKeywords.addAll(keywords);
    }

    public void excludeKeywords(List<String> keywords) {
        this.excludedKeywords.addAll(keywords);
    }

    public void setKeywords(List<String> keywords) {
        this.keywords.clear();
        this.keywords.addAll(keywords);
    }

    public List<String> getKeywords() {
        return Collections.unmodifiableList(this.keywords);
    }

    public List<String> getIncludedKeywords() {
        return Collections.unmodifiableList(this.includedKeywords);
    }

    public List<String> getExcludedKeywords() {
        return Collections.unmodifiableList(this.excludedKeywords);
    }

    public List<String> getEffectiveKeywords() {
        if (this.keywords.isEmpty()) return this.doGetEffectiveKeywords();
        else return this.getKeywords();
    }

    protected List<String> doGetEffectiveKeywords() {
        List<String> keywords = this.getParentSymbols().stream()
                .flatMap(parent -> parent.getEffectiveKeywords().stream())
                .collect(Collectors.toList());

        keywords.addAll(this.includedKeywords);
        keywords.removeAll(this.excludedKeywords);

        return keywords;
    }
}
