/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.option._symboltable;

import de.monticore.lang.monticar.sol.grammars.common._ast.ASTArrayType;
import de.monticore.lang.monticar.sol.grammars.common._ast.ASTPrimitiveType;
import de.monticore.lang.monticar.sol.grammars.common._symboltable.SymbolWithType;
import de.monticore.lang.monticar.sol.grammars.common._symboltable.SymbolWithValue;
import de.monticore.lang.monticar.sol.grammars.common._symboltable.SymbolWithValueList;
import de.monticore.lang.monticar.sol.grammars.option._ast.*;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.assignment.LiteralAssignmentSymbol;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.assignment.LiteralListAssignmentSymbol;
import de.monticore.mcliterals._ast.*;
import de.monticore.symboltable.ArtifactScope;
import de.monticore.symboltable.ImportStatement;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.se_rwth.commons.Names;

import java.util.*;

public class OptionSymbolTableCreator extends OptionSymbolTableCreatorTOP {
    protected final Deque<OptionTypeSymbol> types;
    protected final Deque<SymbolWithType> withTypes;
    protected final Deque<SymbolWithValue> withValues;
    protected final Deque<SymbolWithValueList> withValueLists;
    protected final Deque<SymbolWithOptions> withOptions;
    protected final Deque<OptionSymbol> options;

    public OptionSymbolTableCreator(ResolvingConfiguration resolvingConfig, MutableScope enclosingScope) {
        super(resolvingConfig, enclosingScope);

        this.types = new ArrayDeque<>();
        this.withTypes = new ArrayDeque<>();
        this.withValues = new ArrayDeque<>();
        this.withValueLists = new ArrayDeque<>();
        this.withOptions = new ArrayDeque<>();
        this.options = new ArrayDeque<>();
    }

    public OptionSymbolTableCreator(ResolvingConfiguration resolvingConfig, Deque<MutableScope> scopeStack) {
        super(resolvingConfig, scopeStack);

        this.types = new ArrayDeque<>();
        this.withTypes = new ArrayDeque<>();
        this.withValues = new ArrayDeque<>();
        this.withValueLists = new ArrayDeque<>();
        this.withOptions = new ArrayDeque<>();
        this.options = new ArrayDeque<>();
    }

    protected MutableScope scope() {
        return this.currentScope().orElse(null);
    }

    protected Optional<OptionTypeSymbol> type() {
        return this.types.isEmpty() ? Optional.empty() : Optional.of(this.types.peek());
    }

    protected Optional<SymbolWithType> withType() {
        return this.withTypes.isEmpty() ? Optional.empty() : Optional.of(this.withTypes.peek());
    }

    protected Optional<OptionSymbol> option() {
        return this.options.isEmpty() ? Optional.empty() : Optional.of(this.options.peek());
    }

    protected Optional<SymbolWithValue> withValue() {
        return this.withValues.isEmpty() ? Optional.empty() : Optional.of(this.withValues.peek());
    }

    protected Optional<SymbolWithValueList> withValueList() {
        return this.withValueLists.isEmpty() ? Optional.empty() : Optional.of(this.withValueLists.peek());
    }

    protected Optional<SymbolWithOptions> withOption() {
        return this.withOptions.isEmpty() ? Optional.empty() : Optional.of(this.withOptions.peek());
    }

    protected void addOrSetValue(Object value) {
        this.withValue().ifPresent(withValue -> withValue.setValue(value));
        this.withValueList().ifPresent(withValueList -> withValueList.addValue(value));
    }

    @Override
    protected PropAssignmentSymbol create_PropAssignment(ASTPropAssignment ast) {
        if (ast.isPresentLiteral()) return new LiteralAssignmentSymbol(ast.getName());
        else return new LiteralListAssignmentSymbol(ast.getName());
    }

    @Override
    protected void initialize_OptionType(OptionTypeSymbol optionType, ASTOptionType ast) {
        optionType.setEnclosingScope(this.scope());
        ast.setOptionTypeSymbol(optionType);
        this.types.push(optionType);
    }

    @Override
    protected void initialize_PropDeclaration(PropDeclarationSymbol declaration, ASTPropDeclaration ast) {
        MutableScope currentScope = this.scope();
        PropDeclarationSymbolReference reference = new PropDeclarationSymbolReference(declaration.getName(), currentScope);

        declaration.setEnclosingScope(currentScope);
        declaration.setRequired(ast.isRequired());
        ast.setPropDeclarationSymbol(declaration);
        this.withTypes.push(declaration);
        this.type().ifPresent(type -> type.addDeclaration(reference));
    }

    @Override
    protected void initialize_Option(OptionSymbol option, ASTOption ast) {
        MutableScope currentScope = this.scope();
        OptionTypeSymbolReference type = new OptionTypeSymbolReference(ast.getType(), currentScope);
        OptionSymbolReference reference = new OptionSymbolReference(ast.getName(), currentScope);

        option.setType(type);
        option.setEnclosingScope(currentScope);
        ast.setOptionSymbol(option);
        this.withOption().ifPresent(withOption -> withOption.addOption(reference));
        this.options.push(option);
        this.withOptions.push(option);
    }

    @Override
    protected void initialize_PropAssignment(PropAssignmentSymbol assignment, ASTPropAssignment ast) {
        MutableScope currentScope = this.scope();
        PropAssignmentSymbolReference reference = new PropAssignmentSymbolReference(ast.getName(), currentScope);

        assignment.setEnclosingScope(currentScope);
        ast.setPropAssignmentSymbol(assignment);
        assignment.asLiteralAssignment().ifPresent(this.withValues::push);
        assignment.asLiteralListAssignment().ifPresent(this.withValueLists::push);
        this.option().ifPresent(option -> {
            option.addAssignment(reference);
            assignment.setOwner(new OptionSymbolReference(option.getName(), currentScope));
        });
    }

    @Override
    public void visit(ASTOptionCompilationUnit node) {
        List<ImportStatement> imports = new ArrayList<>();
        String packageName = Names.getQualifiedName(node.getPackageList());

        this.putOnStack(new ArtifactScope(Optional.empty(), packageName, imports));
    }

    @Override
    public void visit(ASTComposite node) {
        OptionLiterals composite = node.getCompositeValue();

        super.visit(node);
        this.type().ifPresent(type -> type.setComposite(composite));
    }

    @Override
    public void visit(ASTStringLiteral node) {
        super.visit(node);
        this.addOrSetValue(node.getValue());
    }

    @Override
    public void visit(ASTBooleanLiteral node) {
        super.visit(node);
        this.addOrSetValue(node.getValue());
    }

    @Override
    public void visit(ASTSignedIntLiteral node) {
        super.visit(node);
        this.addOrSetValue(node.getValue());
    }

    @Override
    public void visit(ASTSignedLongLiteral node) {
        super.visit(node);
        this.addOrSetValue(node.getValue());
    }

    @Override
    public void visit(ASTSignedFloatLiteral node) {
        super.visit(node);
        this.addOrSetValue(node.getValue());
    }

    @Override
    public void visit(ASTSignedDoubleLiteral node) {
        super.visit(node);
        this.addOrSetValue(node.getValue());
    }

    @Override
    public void visit(ASTPrimitiveType node) {
        String type = node.getIdentifier();

        this.withType().ifPresent(withType -> withType.setType(type));
        this.withValue().ifPresent(withValue -> withValue.setType(type));
        this.withValueList().ifPresent(withValue -> withValue.setType(type));
    }

    @Override
    public void visit(ASTArrayType node) {
        String type = node.getIdentifier();

        this.withType().ifPresent(withType -> withType.setType(type));
        this.withValue().ifPresent(withValue -> withValue.setType(type));
        this.withValueList().ifPresent(withValue -> withValue.setType(type));
    }

    @Override
    public void visit(ASTReturn node) {
        String value = node.getTypeIdentifier();

        super.visit(node);
        this.type().ifPresent(type -> type.setReturnType(value));
    }

    @Override
    public void endVisit(ASTOptionType node) {
        this.types.pop();
    }

    @Override
    public void endVisit(ASTOption ast) {
        super.endVisit(ast);
        this.options.pop();
        this.withOptions.pop();
    }
}
