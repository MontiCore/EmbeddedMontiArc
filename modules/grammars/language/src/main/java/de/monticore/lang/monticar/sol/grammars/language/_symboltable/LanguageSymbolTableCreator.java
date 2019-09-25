/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.language._symboltable;

import de.monticore.lang.monticar.sol.grammars.language._ast.*;
import de.monticore.lang.monticar.sol.grammars.options._ast.ASTOption;
import de.monticore.lang.monticar.sol.grammars.options._symboltable.OptionSymbolReference;
import de.monticore.lang.monticar.sol.grammars.options._symboltable.OptionsSymbolTableCreator;
import de.monticore.mcliterals._ast.ASTBooleanLiteral;
import de.monticore.mcliterals._ast.ASTDoubleLiteral;
import de.monticore.mcliterals._ast.ASTStringLiteral;
import de.monticore.symboltable.*;
import de.se_rwth.commons.Names;

import java.util.ArrayList;
import java.util.Deque;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;

public class LanguageSymbolTableCreator extends LanguageSymbolTableCreatorTOP {
    protected final OptionsSymbolTableCreator optionsSTC;

    protected LanguageSymbol language;
    protected TemplateAttributeSymbol attribute;

    public LanguageSymbolTableCreator(ResolvingConfiguration resolvingConfig, MutableScope enclosingScope) {
        super(resolvingConfig, enclosingScope);

        this.optionsSTC = new OptionsSymbolTableCreator(resolvingConfig, this.scopeStack);
    }

    public LanguageSymbolTableCreator(ResolvingConfiguration resolvingConfig, Deque<MutableScope> scopeStack) {
        super(resolvingConfig, scopeStack);

        this.optionsSTC = new OptionsSymbolTableCreator(resolvingConfig, this.scopeStack);
    }

    protected Optional<LanguageSymbol> currentLanguage() {
        return Optional.ofNullable(this.language);
    }

    protected Optional<TemplateDeclarationSymbol> currentDeclaration() {
        return this.currentSymbol()
                .filter(symbol -> symbol instanceof TemplateDeclarationSymbol)
                .map(symbol -> (TemplateDeclarationSymbol)symbol);
    }

    protected Optional<TemplateAttributeSymbol> currentAttribute() {
        return Optional.ofNullable(this.attribute);
    }

    protected void consumeAttribute(Consumer<TemplateAttributeSymbol> consumer) {
        this.currentAttribute().ifPresent(consumer);

        this.attribute = null;
    }

    @Override
    protected void initialize_Language(LanguageSymbol language, ASTLanguage ast) {
        MutableScope currentScope = this.currentScope().orElse(null);

        this.language = language;

        language.setEnclosingScope(currentScope);
        ast.setLanguageSymbol(language);
        ast.getParentList().forEach(parent -> {
            LanguageSymbolReference reference = new LanguageSymbolReference(parent, currentScope);

            this.language.addParent(reference);
        });
    }

    @Override
    protected void initialize_TemplateAttribute(TemplateAttributeSymbol attribute, ASTTemplateAttribute ast) {
        MutableScope currentScope = this.currentScope().orElse(null);

        this.attribute = attribute;

        ast.setTemplateAttributeSymbol(attribute);
        attribute.setEnclosingScope(currentScope);
        this.currentDeclaration().ifPresent(declaration -> {
            TemplateAttributeSymbolReference reference =
                    new TemplateAttributeSymbolReference(attribute.getName(), currentScope);

            declaration.addAttribute(reference);
        });
    }

    @Override
    protected void initialize_TemplateDeclaration(TemplateDeclarationSymbol declaration, ASTTemplateDeclaration ast) {
        MutableScope currentScope = this.currentScope().orElse(null);

        ast.setTemplateDeclarationSymbol(declaration);
        declaration.setPath(ast.getPath().getValue());
        declaration.setEnclosingScope(currentScope);
        this.currentLanguage().ifPresent(language -> {
            TemplateDeclarationSymbolReference reference =
                    new TemplateDeclarationSymbolReference(declaration.getName(), currentScope);

            language.addLocalDeclaration(reference);
        });
    }

    @Override
    protected void initialize_TemplateUndeclaration(TemplateUndeclarationSymbol undeclaration, ASTTemplateUndeclaration ast) {
        MutableScope currentScope = this.currentScope().orElse(null);

        ast.setTemplateUndeclarationSymbol(undeclaration);
        undeclaration.setEnclosingScope(currentScope);
        this.currentLanguage().ifPresent(language -> {
            TemplateUndeclarationSymbolReference reference =
                    new TemplateUndeclarationSymbolReference(undeclaration.getName(), currentScope);

            undeclaration.setLanguageSymbol(language);
            language.addLocalUndeclaration(reference);
        });
    }

    @Override
    public void visit(ASTLanguageCompilationUnit node) {
        List<ImportStatement> imports = new ArrayList<>();
        String packageName = Names.getQualifiedName(node.getPackageList());

        node.forEachImports(i -> {
            String qualifiedName = Names.getQualifiedName(i.getImportList());
            ImportStatement statement = new ImportStatement(qualifiedName, i.isStar());

            imports.add(statement);
        });

        this.putOnStack(new ArtifactScope(Optional.empty(), packageName, imports));
    }

    @Override
    public void handle(ASTOption node) {
        this.optionsSTC.handle(node);
        this.currentDeclaration().ifPresent(declaration -> {
            OptionSymbolReference reference =
                    new OptionSymbolReference(node.getOptionSymbol().getName(), this.currentScope().orElse(null));

            declaration.addOption(reference);
        });
    }

    @Override
    public void visit(ASTStringLiteral node) {
        this.consumeAttribute(attribute -> attribute.setValue(node.getValue()));
    }

    @Override
    public void visit(ASTBooleanLiteral node) {
        this.consumeAttribute(attribute -> attribute.setValue(node.getValue()));
    }

    @Override
    public void visit(ASTDoubleLiteral node) {
        this.consumeAttribute(attribute -> attribute.setValue(node.getValue()));
    }
}
