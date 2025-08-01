/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.artifact._symboltable;

import de.monticore.lang.monticar.sol.grammars.common._symboltable.SymbolTableCreator;
import de.monticore.lang.monticar.sol.grammars.common._symboltable.SymbolWithProjectPath;
import de.monticore.lang.monticar.sol.grammars.environment._symboltable.DockerfileSymbolReference;
import de.monticore.lang.monticar.sol.grammars.environment._symboltable.SymbolWithEnvironment;
import de.monticore.lang.monticar.sol.grammars.language._symboltable.LanguageSymbolReference;
import de.monticore.lang.monticar.sol.grammars.language._symboltable.SymbolWithLanguages;
import de.monticore.lang.monticar.sol.grammars.artifact._ast.*;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;

import java.util.*;

public class ArtifactSymbolTableCreator extends ArtifactSymbolTableCreatorTOP implements SymbolTableCreator {
    protected final Deque<ToolSymbol> tools;
    protected final Deque<SymbolWithAlias> withAliases;
    protected final Deque<SymbolWithArtifacts> withArtifacts;
    protected final Deque<SymbolWithEnvironment> withEnvironments;
    protected final Deque<SymbolWithLanguages> withLanguages;
    protected final Deque<SymbolWithProjectPath> withProjectPaths;

    public ArtifactSymbolTableCreator(ResolvingConfiguration resolvingConfig, MutableScope enclosingScope) {
        super(resolvingConfig, enclosingScope);

        this.tools = new ArrayDeque<>();
        this.withAliases = new ArrayDeque<>();
        this.withArtifacts = new ArrayDeque<>();
        this.withEnvironments = new ArrayDeque<>();
        this.withLanguages = new ArrayDeque<>();
        this.withProjectPaths = new ArrayDeque<>();
    }

    public ArtifactSymbolTableCreator(ResolvingConfiguration resolvingConfig, Deque<MutableScope> scopeStack) {
        super(resolvingConfig, scopeStack);

        this.tools = new ArrayDeque<>();
        this.withAliases = new ArrayDeque<>();
        this.withArtifacts = new ArrayDeque<>();
        this.withEnvironments = new ArrayDeque<>();
        this.withLanguages = new ArrayDeque<>();
        this.withProjectPaths = new ArrayDeque<>();
    }

    protected MutableScope scope() {
        return this.currentScope().orElse(null);
    }

    protected Optional<ToolSymbol> tool() {
        return this.tools.isEmpty() ? Optional.empty() : Optional.of(this.tools.peek());
    }

    protected Optional<SymbolWithAlias> withAlias() {
        return this.withAliases.isEmpty() ? Optional.empty() : Optional.of(this.withAliases.peek());
    }

    protected Optional<SymbolWithArtifacts> withArtifact() {
        return this.withArtifacts.isEmpty() ? Optional.empty() : Optional.of(this.withArtifacts.peek());
    }

    protected Optional<SymbolWithEnvironment> withEnvironment() {
        return this.withEnvironments.isEmpty() ? Optional.empty() : Optional.of(this.withEnvironments.peek());
    }

    protected Optional<SymbolWithLanguages> withLanguage() {
        return this.withLanguages.isEmpty() ? Optional.empty() : Optional.of(this.withLanguages.peek());
    }

    protected Optional<SymbolWithProjectPath> withProjectPath() {
        return this.withProjectPaths.isEmpty() ? Optional.empty() : Optional.of(this.withProjectPaths.peek());
    }

    @Override
    protected void initialize_Tool(ToolSymbol tool, ASTTool ast) {
        tool.setVirtual(ast.isVirtual());
        ast.setToolSymbol(tool);
        tool.setEnclosingScope(this.scope());
        this.tools.push(tool);
        this.withAliases.push(tool);
        this.withArtifacts.push(tool);
        this.withEnvironments.push(tool);
        this.withLanguages.push(tool);
        this.withProjectPaths.push(tool);
    }

    @Override
    protected void initialize_Product(ProductSymbol product, ASTProduct ast) {
        ast.setProductSymbol(product);
        product.setEnclosingScope(this.scope());
        this.withAliases.push(product);
        this.withArtifacts.push(product);
        this.withEnvironments.push(product);
        this.withLanguages.push(product);
        this.withProjectPaths.push(product);
    }

    @Override
    protected void initialize_Artifact(ArtifactSymbol artifact, ASTArtifact ast) {
        MutableScope currentScope = this.scope();
        ArtifactSymbolReference reference = new ArtifactSymbolReference(artifact.getName(), currentScope);

        ast.setArtifactSymbol(artifact);
        artifact.setEnclosingScope(currentScope);
        this.withAliases.push(artifact);
        this.withProjectPaths.push(artifact);
        this.withArtifact().ifPresent(withArtifact -> withArtifact.addArtifact(reference));
    }

    @Override
    public void visit(ASTArtifactCompilationUnit node) {
        this.putOnStack(node.getPackageList(), node.getImportList());
    }

    @Override
    public void visit(ASTAlias node) {
        String alias = node.getIdentifier();

        this.withAlias().ifPresent(withAlias -> withAlias.setAlias(alias));
    }

    @Override
    public void visit(ASTPath node) {
        this.withProjectPath().ifPresent(withProjectPath -> {
            withProjectPath.setOrigin(node.getOrigin());
            withProjectPath.setPath(node.getRelativePath());
        });
    }

    @Override
    public void visit(ASTCommand node) {
        this.tool().ifPresent(tool -> {
            if (node.isPresentCommand()) tool.setCommand(node.getCommandValue());
            else if (node.isPresentPrefix()) tool.setPrefix(node.getPrefixValue());
            else if (node.isPresentSuffix()) tool.setSuffix(node.getSuffixValue());
        });
    }

    @Override
    public void visit(ASTEnvironment node) {
        DockerfileSymbolReference environment = new DockerfileSymbolReference(node.getIdentifier(), this.scope());

        this.withEnvironment().ifPresent(withEnvironment -> withEnvironment.setEnvironment(environment));
    }

    @Override
    public void visit(ASTLanguage node) {
        MutableScope currentScope = this.scope();
        List<String> names = node.isPresentLanguage() ? Collections.singletonList(node.getLanguage()) : node.getLanguageList();

        names.forEach(name -> {
            LanguageSymbolReference language = new LanguageSymbolReference(name, currentScope);

            this.withLanguage().ifPresent(withLanguage -> withLanguage.addLanguage(language));
        });
    }

    @Override
    public void endVisit(ASTTool ast) {
        this.tools.pop();
        this.withAliases.pop();
        this.withArtifacts.pop();
        this.withEnvironments.pop();
        this.withLanguages.pop();
        this.withProjectPaths.pop();
    }

    @Override
    public void endVisit(ASTProduct ast) {
        this.withAliases.pop();
        this.withArtifacts.pop();
        this.withEnvironments.pop();
        this.withLanguages.pop();
        this.withProjectPaths.pop();
    }

    @Override
    public void endVisit(ASTArtifact ast) {
        this.withAliases.pop();
        this.withProjectPaths.pop();
    }
}
