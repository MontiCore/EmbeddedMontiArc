/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.language._symboltable;

import de.monticore.lang.monticar.sol.grammars.common._symboltable.SymbolTableCreator;
import de.monticore.lang.monticar.sol.grammars.language._ast.*;
import de.monticore.lang.monticar.sol.grammars.option._ast.ASTOption;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.OptionSymbolReference;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.OptionSymbolTableCreator;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.SymbolWithOptions;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.List;
import java.util.Optional;

public class LanguageSymbolTableCreator extends LanguageSymbolTableCreatorTOP implements SymbolTableCreator {
    protected final Deque<LanguageSymbol> languages;
    protected final Deque<TemplateDeclarationSymbol> declarations;
    protected final Deque<SymbolWithOptions> withOptions;

    protected final OptionSymbolTableCreator optionSTC;

    public LanguageSymbolTableCreator(ResolvingConfiguration resolvingConfig, MutableScope enclosingScope) {
        super(resolvingConfig, enclosingScope);

        this.languages = new ArrayDeque<>();
        this.declarations = new ArrayDeque<>();
        this.withOptions = new ArrayDeque<>();

        this.optionSTC = new OptionSymbolTableCreator(resolvingConfig, this.scopeStack);
    }

    public LanguageSymbolTableCreator(ResolvingConfiguration resolvingConfig, Deque<MutableScope> scopeStack) {
        super(resolvingConfig, scopeStack);

        this.languages = new ArrayDeque<>();
        this.declarations = new ArrayDeque<>();
        this.withOptions = new ArrayDeque<>();

        this.optionSTC = new OptionSymbolTableCreator(resolvingConfig, this.scopeStack);
    }

    protected MutableScope scope() {
        return this.currentScope().orElse(null);
    }

    protected Optional<LanguageSymbol> language() {
        return this.languages.isEmpty() ? Optional.empty() : Optional.ofNullable(this.languages.peek());
    }

    protected Optional<TemplateDeclarationSymbol> declaration() {
        return this.declarations.isEmpty() ? Optional.empty() : Optional.ofNullable(this.declarations.peek());
    }

    protected Optional<SymbolWithOptions> withOption() {
        return this.withOptions.isEmpty() ? Optional.empty() : Optional.ofNullable(this.withOptions.peek());
    }

    @Override
    protected void initialize_Language(LanguageSymbol language, ASTLanguage ast) {
        MutableScope currentScope = this.scope();

        this.languages.push(language);
        language.setEnclosingScope(currentScope);
        ast.setLanguageSymbol(language);
        ast.getParentList().forEach(parent -> language.addParent(new LanguageSymbolReference(parent, currentScope)));
    }

    @Override
    protected void initialize_TemplateDeclaration(TemplateDeclarationSymbol declaration, ASTTemplateDeclaration ast) {
        MutableScope currentScope = this.scope();
        TemplateDeclarationSymbolReference reference = new TemplateDeclarationSymbolReference(declaration.getName(), currentScope);

        ast.setTemplateDeclarationSymbol(declaration);
        declaration.setEnclosingScope(currentScope);
        this.withOptions.push(declaration);
        this.declarations.push(declaration);
        this.language().ifPresent(language -> language.addLocalDeclaration(reference));
    }

    @Override
    protected void initialize_TemplateExclusion(TemplateExclusionSymbol undeclaration, ASTTemplateExclusion ast) {
        MutableScope currentScope = this.scope();
        TemplateExclusionSymbolReference reference = new TemplateExclusionSymbolReference(undeclaration.getName(), currentScope);

        ast.setTemplateExclusionSymbol(undeclaration);
        undeclaration.setEnclosingScope(currentScope);
        this.language().ifPresent(language -> {
            LanguageSymbolReference languageReference = new LanguageSymbolReference(language.getName(), language.getEnclosingScope());

            undeclaration.setOwner(languageReference);
            language.addLocalUndeclaration(reference);
        });
    }

    @Override
    public void handle(ASTOption node) {
        MutableScope currentScope = this.scope();
        OptionSymbolReference reference = new OptionSymbolReference(node.getName(), currentScope);

        this.optionSTC.handle(node);
        this.withOption().ifPresent(withOption -> withOption.addOption(reference));
    }

    @Override
    public void visit(ASTLanguageCompilationUnit node) {
        this.putOnStack(node.getPackageList(), node.getImportList());
    }

    @Override
    public void visit(ASTKeywords node) {
        super.visit(node);
        this.language().ifPresent(language -> {
            List<String> keywords = node.getKeywords();

            if (node.isInclusion()) language.includeKeywords(keywords);
            else if (node.isExclusion()) language.excludeKeywords(keywords);
            else language.setKeywords(keywords);
        });
    }

    @Override
    public void visit(ASTServer node) {
        super.visit(node);
        this.language().ifPresent(language -> {
            language.setOrigin(node.getOrigin());
            language.setPath(node.getRelativePath());
        });
    }

    @Override
    public void visit(ASTTemplateAttribute node) {
        this.declaration().ifPresent(declaration -> {
            String value = node.getValue();

            if (node.isPath()) declaration.setPath(value);
            else if (node.isLabel()) declaration.setLabel(value);
        });
    }

    @Override
    public void visit(ASTExtension node) {
        String extension = node.getValue();

        super.visit(node);
        this.language().ifPresent(language -> language.setExtension(extension));
    }

    @Override
    public void endVisit(ASTLanguage node) {
        this.languages.pop();
    }

    @Override
    public void endVisit(ASTTemplateDeclaration node) {
        super.endVisit(node);
        this.withOptions.pop();
        this.declarations.pop();
    }
}
