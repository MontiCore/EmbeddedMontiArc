/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.ide._symboltable;

import de.monticore.lang.monticar.sol.grammars.common._ast.ASTArrayType;
import de.monticore.lang.monticar.sol.grammars.common._ast.ASTPrimitiveType;
import de.monticore.lang.monticar.sol.grammars.common._symboltable.SymbolTableCreator;
import de.monticore.lang.monticar.sol.grammars.common._symboltable.SymbolWithValue;
import de.monticore.lang.monticar.sol.grammars.common._symboltable.SymbolWithValueList;
import de.monticore.lang.monticar.sol.grammars.ide._ast.*;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.fill.LiteralFillSymbol;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.fill.LiteralListFillSymbol;
import de.monticore.lang.monticar.sol.grammars.option._ast.ASTOption;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.OptionSymbolReference;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.OptionSymbolTableCreator;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.SymbolWithOptions;
import de.monticore.lang.monticar.sol.grammars.artifact._symboltable.ProductSymbolReference;
import de.monticore.lang.monticar.sol.grammars.artifact._symboltable.ToolSymbolReference;
import de.monticore.mcliterals._ast.*;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;

import java.util.*;
import java.util.function.Consumer;

public class IDESymbolTableCreator extends IDESymbolTableCreatorTOP implements SymbolTableCreator {
    protected final OptionSymbolTableCreator optionsSTC;

    protected final Deque<IDESymbol> ides;
    protected final Deque<ConfigurationSymbol> configurations;
    protected final Deque<ConfigurationTypeSymbol> configurationTypes;
    protected final Deque<ModuleSymbol> modules;
    protected final Deque<ModuleTypeSymbol> moduleTypes;
    protected final Deque<OptionFillSymbol> fills;
    protected final Deque<TaskSymbol> tasks;
    protected final Deque<WriteSymbol> writes;
    protected final Deque<SymbolWithConfigurations> withConfigurations;
    protected final Deque<SymbolWithFilledOptions> withFilledOptions;
    protected final Deque<SymbolWithInheritedOptions> withInheritedOptions;
    protected final Deque<SymbolWithTypeAttribute> withTypeAttributes;
    protected final Deque<SymbolWithOptions> withOptions;
    protected final Deque<SymbolWithValue> withValues;
    protected final Deque<SymbolWithValueList> withValueLists;

    public IDESymbolTableCreator(ResolvingConfiguration resolvingConfig, MutableScope enclosingScope) {
        super(resolvingConfig, enclosingScope);

        this.optionsSTC = new OptionSymbolTableCreator(resolvingConfig, this.scopeStack);

        this.ides = new ArrayDeque<>();
        this.configurations = new ArrayDeque<>();
        this.configurationTypes = new ArrayDeque<>();
        this.modules = new ArrayDeque<>();
        this.moduleTypes = new ArrayDeque<>();
        this.fills = new ArrayDeque<>();
        this.tasks = new ArrayDeque<>();
        this.writes = new ArrayDeque<>();
        this.withConfigurations = new ArrayDeque<>();
        this.withFilledOptions = new ArrayDeque<>();
        this.withInheritedOptions = new ArrayDeque<>();
        this.withTypeAttributes = new ArrayDeque<>();
        this.withOptions = new ArrayDeque<>();
        this.withValues = new ArrayDeque<>();
        this.withValueLists = new ArrayDeque<>();
    }

    public IDESymbolTableCreator(ResolvingConfiguration resolvingConfig, Deque<MutableScope> scopeStack) {
        super(resolvingConfig, scopeStack);

        this.optionsSTC = new OptionSymbolTableCreator(resolvingConfig, this.scopeStack);

        this.ides = new ArrayDeque<>();
        this.configurations = new ArrayDeque<>();
        this.configurationTypes = new ArrayDeque<>();
        this.modules = new ArrayDeque<>();
        this.moduleTypes = new ArrayDeque<>();
        this.fills = new ArrayDeque<>();
        this.tasks = new ArrayDeque<>();
        this.writes = new ArrayDeque<>();
        this.withConfigurations = new ArrayDeque<>();
        this.withFilledOptions = new ArrayDeque<>();
        this.withInheritedOptions = new ArrayDeque<>();
        this.withTypeAttributes = new ArrayDeque<>();
        this.withOptions = new ArrayDeque<>();
        this.withValues = new ArrayDeque<>();
        this.withValueLists = new ArrayDeque<>();
    }

    protected MutableScope scope() {
        return this.currentScope().orElse(null);
    }

    protected Optional<IDESymbol> ide() {
        return this.ides.isEmpty() ? Optional.empty() : Optional.of(this.ides.peek());
    }

    protected Optional<ConfigurationSymbol> configuration() {
        return this.configurations.isEmpty() ? Optional.empty() : Optional.of(this.configurations.peek());
    }

    protected Optional<ConfigurationTypeSymbol> configurationType() {
        return this.configurationTypes.isEmpty() ? Optional.empty() : Optional.of(this.configurationTypes.peek());
    }

    protected Optional<ModuleSymbol> module() {
        return this.modules.isEmpty() ? Optional.empty() : Optional.of(this.modules.peek());
    }

    protected Optional<ModuleTypeSymbol> moduleType() {
        return this.moduleTypes.isEmpty() ? Optional.empty() : Optional.of(this.moduleTypes.peek());
    }

    protected Optional<OptionFillSymbol> fill() {
        return this.fills.isEmpty() ? Optional.empty() : Optional.of(this.fills.peek());
    }

    protected Optional<TaskSymbol> task() {
        return this.tasks.isEmpty() ? Optional.empty() : Optional.of(this.tasks.peek());
    }

    protected Optional<WriteSymbol> write() {
        return this.writes.isEmpty() ? Optional.empty() : Optional.of(this.writes.peek());
    }

    protected Optional<SymbolWithConfigurations> withConfiguration() {
        return this.withConfigurations.isEmpty() ? Optional.empty() : Optional.of(this.withConfigurations.peek());
    }

    protected Optional<SymbolWithFilledOptions> withFilledOption() {
        return this.withFilledOptions.isEmpty() ? Optional.empty() : Optional.of(this.withFilledOptions.peek());
    }

    protected Optional<SymbolWithInheritedOptions> withInheritedOption() {
        return this.withInheritedOptions.isEmpty() ? Optional.empty() : Optional.of(this.withInheritedOptions.peek());
    }

    protected Optional<SymbolWithTypeAttribute> withTypeAttribute() {
        return this.withTypeAttributes.isEmpty() ? Optional.empty() : Optional.of(this.withTypeAttributes.peek());
    }

    protected Optional<SymbolWithOptions> withOption() {
        return this.withOptions.isEmpty() ? Optional.empty() : Optional.of(this.withOptions.peek());
    }

    protected Optional<SymbolWithValue> withValue() {
        return this.withValues.isEmpty() ? Optional.empty() : Optional.of(this.withValues.peek());
    }

    protected Optional<SymbolWithValueList> withValueList() {
        return this.withValueLists.isEmpty() ? Optional.empty() : Optional.of(this.withValueLists.peek());
    }

    @Override
    protected void initialize_IDE(IDESymbol ide, ASTIDE ast) {
        MutableScope currentScope = this.scope();
        Consumer<String> addParent = parent -> ide.addParent(new IDESymbolReference(parent, currentScope));

        ide.setOnline(ast.isOnline());
        ide.setEnclosingScope(currentScope);
        ast.setIDESymbol(ide);
        ast.forEachParents(addParent);
        this.ides.push(ide);
    }

    @Override
    protected OptionFillSymbol create_OptionFill(ASTOptionFill ast) {
        if (ast.isPresentLiteral()) return new LiteralFillSymbol(ast.getName());
        else if (ast.isPresentLiteralList()) return new LiteralListFillSymbol(ast.getName());
        else return super.create_OptionFill(ast);
    }

    @Override
    protected void initialize_ConfigurationType(ConfigurationTypeSymbol configurationType, ASTConfigurationType ast) {
        configurationType.setComponent(ast.isComponent());
        configurationType.setEnclosingScope(this.scope());
        ast.setConfigurationTypeSymbol(configurationType);
        this.withOptions.push(configurationType);
        this.configurationTypes.push(configurationType);
        this.withTypeAttributes.push(configurationType);
        this.withConfigurations.push(configurationType);
    }

    @Override
    protected void initialize_ModuleType(ModuleTypeSymbol moduleType, ASTModuleType ast) {
        moduleType.setEnclosingScope(this.scope());
        ast.setModuleTypeSymbol(moduleType);
        this.withOptions.push(moduleType);
        this.moduleTypes.push(moduleType);
        this.withConfigurations.push(moduleType);
        this.withTypeAttributes.push(moduleType);
    }

    @Override
    protected void initialize_Configuration(ConfigurationSymbol configuration, ASTConfiguration ast) {
        MutableScope currentScope = this.scope();
        ConfigurationSymbolReference reference = new ConfigurationSymbolReference(ast.getName(), currentScope);
        ConfigurationTypeSymbolReference type = new ConfigurationTypeSymbolReference(ast.getType(), currentScope);

        configuration.setEnclosingScope(currentScope);
        configuration.setType(type);
        ast.setConfigurationSymbol(configuration);
        this.configurations.push(configuration);
        this.withInheritedOptions.push(configuration);
        this.withFilledOptions.push(configuration);
        this.withConfiguration().ifPresent(withConfiguration -> withConfiguration.addConfiguration(reference));
    }

    @Override
    protected void initialize_Module(ModuleSymbol module, ASTModule ast) {
        MutableScope currentScope = this.scope();
        ModuleSymbolReference reference = new ModuleSymbolReference(ast.getName(), currentScope);
        ModuleTypeSymbolReference type = new ModuleTypeSymbolReference(ast.getType(), currentScope);

        module.setEnclosingScope(currentScope);
        module.setType(type);
        ast.setModuleSymbol(module);
        this.withFilledOptions.push(module);
        this.moduleType().ifPresent(moduleType -> moduleType.addModule(reference));
    }

    @Override
    protected void initialize_OptionInherit(OptionInheritSymbol inherit, ASTOptionInherit ast) {
        MutableScope currentScope = this.scope();
        OptionInheritSymbolReference reference = new OptionInheritSymbolReference(ast.getName(), currentScope);
        OptionSymbolReference parent = new OptionSymbolReference(ast.getParent(), currentScope);

        inherit.setEnclosingScope(currentScope);
        inherit.setParent(parent);
        ast.setOptionInheritSymbol(inherit);
        this.withInheritedOption().ifPresent(withInheritedOption -> withInheritedOption.addOptionInherit(reference));
        this.configuration().ifPresent(configuration -> {
            ConfigurationSymbolReference configurationReference =
                    new ConfigurationSymbolReference(configuration.getName(), configuration.getEnclosingScope());

            inherit.setOwner(configurationReference);
        });
    }

    @Override
    protected void initialize_OptionFill(OptionFillSymbol fill, ASTOptionFill ast) {
        MutableScope currentScope = this.scope();
        OptionFillSymbolReference reference = new OptionFillSymbolReference(ast.getName(), currentScope);

        fill.setEnclosingScope(currentScope);
        ast.setOptionFillSymbol(fill);
        fill.asLiteralFill().ifPresent(this.withValues::push);
        fill.asLiteralListFill().ifPresent(this.withValueLists::push);
        this.fills.push(fill);
        this.withFilledOption().ifPresent(withFilledOption -> withFilledOption.addOptionFill(reference));
        this.configuration().ifPresent(configuration -> {
            ConfigurationSymbolReference configurationReference =
                    new ConfigurationSymbolReference(configuration.getName(), configuration.getEnclosingScope());

            fill.setOwner(configurationReference);
        });
    }

    @Override
    protected void initialize_Task(TaskSymbol task, ASTTask ast) {
        MutableScope currentScope = this.scope();
        TaskSymbolReference reference = new TaskSymbolReference(ast.getName(), currentScope);

        ast.setTaskSymbol(task);
        task.setEnclosingScope(currentScope);
        task.setFrontend(ast.isFrontend());
        task.setBackend(ast.isBackend());
        this.tasks.push(task);
        this.configurationType().ifPresent(configurationType -> configurationType.addTask(reference));
        ast.getPredecessorList().forEach(name -> task.addPredecessor(new TaskSymbolReference(name, currentScope)));
    }

    @Override
    protected void initialize_Write(WriteSymbol write, ASTWrite ast) {
        MutableScope currentScope = this.scope();
        WriteSymbolReference reference = new WriteSymbolReference(ast.getName(), currentScope);

        this.writes.push(write);
        this.moduleType().ifPresent(moduleType -> moduleType.addWrite(reference));
    }

    @Override
    public void handle(ASTOption node) {
        OptionSymbolReference reference = new OptionSymbolReference(node.getName(), this.scope());

        this.optionsSTC.handle(node);
        this.withOption().ifPresent(withOption -> withOption.addOption(reference));
    }

    @Override
    public void visit(ASTIDECompilationUnit node) {
        this.putOnStack(node.getPackageList(), node.getImportList());
    }

    @Override
    public void visit(ASTWrite node) {
        String value = node.getPathValue();

        super.visit(node);
        this.write().ifPresent(write -> write.setRelativePath(value));
    }

    @Override
    public void visit(ASTConfigurationName node) {
        String value = node.getNameValue();

        super.visit(node);
        this.configuration().ifPresent(configuration -> configuration.setDisplayName(value));
    }

    @Override
    public void visit(ASTRegistry node) {
        String value = node.getRegistryValue();

        super.visit(node);
        this.ide().ifPresent(ide -> ide.setRegistry(value));
    }

    @Override
    public void visit(ASTBuild node) {
        super.visit(node);
        this.ide().ifPresent(ide -> {
            ide.setBuildPath(node.getRelativePath());
            ide.setBuildOrigin(node.getOrigin());
        });
    }

    @Override
    public void visit(ASTArtifacts node) {
        super.visit(node);

        if (node.isTool()) this.handleTool(node.getIdentifier());
        else if (node.isTools()) node.getIdentifierList().forEach(this::handleTool);
        else if (node.isProduct()) this.handleProduct(node.getIdentifier());
        else if (node.isProducts()) node.getIdentifierList().forEach(this::handleProduct);
    }

    protected void handleTool(String name) {
        ToolSymbolReference reference = new ToolSymbolReference(name, this.scope());

        this.configurationType().ifPresent(configurationType -> configurationType.addTool(reference));
    }

    protected void handleProduct(String name) {
        ProductSymbolReference reference = new ProductSymbolReference(name, this.scope());

        this.configurationType().ifPresent(configurationType -> configurationType.addProduct(reference));
    }

    @Override
    public void visit(ASTContributionTypeAttribute node) {
        String value = node.getLiteralValue();

        super.visit(node);
        this.withTypeAttribute().ifPresent(withTypeAttribute -> {
            if (node.isIcon()) withTypeAttribute.setIcon(value);
            else if (node.isLabel()) withTypeAttribute.setLabel(value);
            else if (node.isCategory()) withTypeAttribute.setCategory(value);
        });
    }

    @Override
    public void visit(ASTContributionType node) {
        List<String> names = node.isPresentIdentifier() ? Collections.singletonList(node.getIdentifier()) : node.getIdentifierList();

        super.visit(node);

        if (node.isModule()) names.forEach(name -> this.handleModule(node, name));
        else if (node.isConfiguration()) names.forEach(name -> this.handleConfiguration(node, name));
    }

    protected void handleModule(ASTContributionType node, String name) {
        ModuleTypeSymbolReference reference = new ModuleTypeSymbolReference(name, this.scope());

        this.ide().ifPresent(ide -> {
            if (node.isInclusion()) ide.includeType(reference);
            else if (node.isExclusion()) ide.excludeType(reference);
        });
    }

    protected void handleConfiguration(ASTContributionType node, String name) {
        ConfigurationTypeSymbolReference reference = new ConfigurationTypeSymbolReference(name, this.scope());

        this.ide().ifPresent(ide -> {
            if (node.isInclusion()) ide.includeType(reference);
            else if (node.isExclusion()) ide.excludeType(reference);
        });
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
        int value = node.getValue();

        super.visit(node);
        this.addOrSetValue(value);
        this.configuration().ifPresent(configuration -> configuration.setOrder(value));
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

    protected void addOrSetValue(Object value) {
        this.withValue().ifPresent(withValue -> withValue.setValue(value));
        this.withValueList().ifPresent(withValueList -> withValueList.addValue(value));
    }

    @Override
    public void visit(ASTPrimitiveType node) {
        String type = node.getIdentifier();

        super.visit(node);
        this.withValue().ifPresent(withValue -> withValue.setType(type));
        this.withValueList().ifPresent(withValueList -> withValueList.setType(type));
    }

    @Override
    public void visit(ASTArrayType node) {
        String type = node.getIdentifier();

        super.visit(node);
        this.withValue().ifPresent(withValue -> withValue.setType(type));
        this.withValueList().ifPresent(withValueList -> withValueList.setType(type));
    }

    @Override
    public void endVisit(ASTConfigurationType ast) {
        this.withOptions.pop();
        this.configurationTypes.pop();
        this.withTypeAttributes.pop();
        this.withConfigurations.pop();
    }

    @Override
    public void endVisit(ASTModuleType ast) {
        this.withOptions.pop();
        this.moduleTypes.pop();
        this.withConfigurations.pop();
        this.withTypeAttributes.pop();
    }

    @Override
    public void endVisit(ASTIDE ast) {
        this.ides.pop();
    }

    @Override
    public void endVisit(ASTOptionFill node) {
        super.endVisit(node);
        this.fills.pop();

        if (!this.withValues.isEmpty()) this.withValues.pop();
        if (!this.withValueLists.isEmpty()) this.withValueLists.pop();
    }

    @Override
    public void endVisit(ASTWrite node) {
        super.endVisit(node);
        this.writes.pop();
    }

    @Override
    public void endVisit(ASTTask node) {
        super.endVisit(node);
        this.tasks.pop();
    }

    @Override
    public void endVisit(ASTModule node) {
        super.endVisit(node);
        this.withFilledOptions.pop();
    }

    @Override
    public void endVisit(ASTConfiguration node) {
        super.endVisit(node);
        this.configurations.pop();
        this.withInheritedOptions.pop();
        this.withFilledOptions.pop();
    }
}
