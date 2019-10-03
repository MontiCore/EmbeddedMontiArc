/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.tool._symboltable;

import de.monticore.lang.monticar.sol.grammars.environment._symboltable.DockerfileSymbolReference;
import de.monticore.lang.monticar.sol.grammars.tool._ast.*;
import de.monticore.lang.monticar.sol.grammars.tool._symboltable.attributes.AliasAttributeSymbol;
import de.monticore.lang.monticar.sol.grammars.tool._symboltable.attributes.EnvironmentAttributeSymbol;
import de.monticore.lang.monticar.sol.grammars.tool._symboltable.attributes.LiteralAttributeSymbol;
import de.monticore.mcliterals._ast.ASTStringLiteral;
import de.monticore.symboltable.ArtifactScope;
import de.monticore.symboltable.ImportStatement;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.se_rwth.commons.Names;

import java.util.*;

public class ToolSymbolTableCreator extends ToolSymbolTableCreatorTOP {
    protected final ToolLiterals[] literals; // Caching as .values() is expensive.
    protected final Deque<AttributableSymbol> attributables;

    protected RootSymbol root;
    protected AttributeSymbol attribute;

    public ToolSymbolTableCreator(ResolvingConfiguration resolvingConfig, MutableScope enclosingScope) {
        super(resolvingConfig, enclosingScope);

        this.literals = ToolLiterals.values();
        this.attributables = new ArrayDeque<>();
    }

    public ToolSymbolTableCreator(ResolvingConfiguration resolvingConfig, Deque<MutableScope> scopeStack) {
        super(resolvingConfig, scopeStack);

        this.literals = ToolLiterals.values();
        this.attributables = new ArrayDeque<>();
    }

    protected Optional<RootSymbol> currentRoot() {
        return Optional.ofNullable(this.root);
    }

    protected Optional<AttributableSymbol> currentAttributable() {
        return this.attributables.isEmpty() ? Optional.empty() : Optional.ofNullable(this.attributables.peek());
    }

    protected Optional<AttributeSymbol> currentAttribute() {
        return Optional.ofNullable(this.attribute);
    }

    @Override
    protected AttributeSymbol create_Attribute(ASTAttribute ast) {
        if (ast.isPresentAlias()) return new AliasAttributeSymbol(ast.getName());
        else if (ast.isPresentLiteral()) return new LiteralAttributeSymbol(ast.getName());
        else if (ast.isPresentEnvironment()) return new EnvironmentAttributeSymbol(ast.getName());
        else return super.create_Attribute(ast);
    }

    @Override
    protected void initialize_Tool(ToolSymbol tool, ASTTool ast) {
        MutableScope currentScope = this.currentScope().orElse(null);

        this.root = tool;

        this.attributables.push(tool);
        tool.setEnclosingScope(currentScope);
        tool.setVirtual(ast.isVirtual());
        ast.setToolSymbol(tool);
    }

    @Override
    protected void initialize_Resources(ResourcesSymbol resources, ASTResources ast) {
        MutableScope currentScope = this.currentScope().orElse(null);

        this.root = resources;

        this.attributables.push(resources);
        resources.setEnclosingScope(currentScope);
        ast.setResourcesSymbol(resources);
    }

    @Override
    protected void initialize_Resource(ResourceSymbol resource, ASTResource ast) {
        MutableScope currentScope = this.currentScope().orElse(null);

        this.attributables.push(resource);
        resource.setEnclosingScope(currentScope);
        ast.setResourceSymbol(resource);
        this.currentRoot().ifPresent(root -> {
            ResourceSymbolReference reference = new ResourceSymbolReference(resource.getName(), currentScope);

            root.addResource(reference);
        });
    }

    @Override
    protected void initialize_Attribute(AttributeSymbol attribute, ASTAttribute ast) {
        MutableScope currentScope = this.currentScope().orElse(null);

        this.doInitializeAttribute(attribute, ast, currentScope);

        if (ast.isPresentEnvironment()) this.doInitializeAttribute((EnvironmentAttributeSymbol) attribute, ast, currentScope);
    }

    protected void doInitializeAttribute(AttributeSymbol attribute, ASTAttribute ast, MutableScope currentScope) {
        this.attribute = attribute;

        ast.setAttributeSymbol(attribute);
        attribute.setEnclosingScope(currentScope);
        this.currentAttributable().ifPresent(attributable -> {
            AttributeSymbolReference reference = new AttributeSymbolReference(attribute.getName(), currentScope);

            attributable.addAttribute(reference);
        });
    }

    protected void doInitializeAttribute(EnvironmentAttributeSymbol attribute, ASTAttribute ast, MutableScope currentScope) {
        DockerfileSymbolReference reference = new DockerfileSymbolReference(ast.getEnvironment(), currentScope);

        attribute.setEnvironment(reference);
    }

    @Override
    public void visit(ASTToolCompilationUnit node) {
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
    public void endVisit(ASTTool ast) {
        this.attributables.pop();
    }

    @Override
    public void endVisit(ASTResources ast) {
        this.attributables.pop();
    }

    @Override
    public void endVisit(ASTResource ast) {
        this.attributables.pop();
    }

    @Override
    public void visit(ASTStringLiteral node) { // TODO: Add BooleanLiteral and Double/IntegerLiteral if necessary.
        String value = node.getValue();

        this.currentAttribute()
                .flatMap(AttributeSymbol::asLiteralAttribute)
                .ifPresent(attribute -> attribute.setValue(value));
    }

    @Override
    public void visit(ASTAlias node) {
        String identifier = node.getIdentifier();
        Optional<AliasAttributeSymbol> symbol = this.currentAttribute().flatMap(AttributeSymbol::asAliasAttribute);

        symbol.ifPresent(attribute -> attribute.setAlias(identifier));
    }
}
