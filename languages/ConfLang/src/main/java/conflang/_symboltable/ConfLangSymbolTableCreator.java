package conflang._symboltable;

import com.google.common.base.Joiner;
import com.google.common.collect.Lists;
import com.google.common.collect.Maps;
import conflang._ast.ASTConfLangCompilationUnit;
import conflang._ast.ASTConfiguration;
import conflang._ast.ASTConfigurationEntry;
import conflang._ast.ASTNestedConfigurationEntry;
import conflang._cocos.ConfLangCoCoChecker;
import conflang._cocos.ConfLangCocoFactory;
import conflangliterals._ast.ASTComponentLiteral;
import conflangliterals._ast.ASTListLiteral;
import conflangliterals._ast.ASTTypelessLiteral;
import de.monticore.mcbasictypes1._ast.ASTQualifiedName;
import de.monticore.mcliterals._ast.*;
import de.monticore.symboltable.*;
import de.monticore.types.types._ast.ASTImportStatement;
import de.se_rwth.commons.Names;
import de.se_rwth.commons.logging.Log;

import java.util.*;

import static conflangliterals.LiteralHelpers.literalValue;

public class ConfLangSymbolTableCreator extends ConfLangSymbolTableCreatorTOP {

    private final ConfLangCoCoChecker checker = ConfLangCocoFactory.createCheckerWithMinimumCoCos();

    private GlobalScope globalScope;

    private String compilationUnitPackage;

    private Map<ASTSignedLiteral, ASTConfigurationEntry> configurationValueMap = Maps.newHashMap();

    private Map<ASTSignedLiteral, ASTNestedConfigurationEntry> nestedConfigurationValueMap = Maps.newHashMap();

    public ConfLangSymbolTableCreator(ResolvingConfiguration resolvingConfig, MutableScope enclosingScope) {
        super(resolvingConfig, enclosingScope);

    }

    public ConfLangSymbolTableCreator(ResolvingConfiguration resolvingConfig, Deque<MutableScope> scopeStack) {
        super(resolvingConfig, scopeStack);
    }

    @Override
    public void visit(ASTConfLangCompilationUnit compilationUnit) {
        super.visit(compilationUnit);

        Optional<? extends MutableScope> mutableScope = currentScope();
        mutableScope.ifPresent(scope -> globalScope = (GlobalScope) scope);

        this.compilationUnitPackage = Names.getQualifiedName(compilationUnit.getPackageList());
        List<ImportStatement> imports = Lists.newArrayList();
        Iterator iterator = compilationUnit.getImportStatementList().iterator();

        while(iterator.hasNext()) {
            ASTImportStatement astImportStatement = (ASTImportStatement)iterator.next();
            String qualifiedImport = Names.getQualifiedName(astImportStatement.getImportList());
            ImportStatement importStatement = new ImportStatement(qualifiedImport, astImportStatement.isStar());
            imports.add(importStatement);
        }

        ArtifactScope artifactScope = new ConfLangArtifactScope(compilationUnitPackage, Collections.emptyList());
        putOnStack(artifactScope);
    }

    @Override
    public void visit(ASTConfigurationEntry ast) {
        super.visit(ast);
        configurationValueMap.put(ast.getValue(), ast);
    }

    @Override
    public void visit(ASTNestedConfigurationEntry ast) {
        super.visit(ast);
        nestedConfigurationValueMap.put(ast.getValue(), ast);
    }

    @Override
    public void visit(ASTCharLiteral node) {
        super.visit(node);
        setSymbolValue(node, node.getValue());
    }

    @Override
    public void visit(ASTComponentLiteral node) {
        super.visit(node);
        ASTQualifiedName qualifiedName = node.getValue();
        List<String> partsList = qualifiedName.getPartList();
        String value = Joiner.on('.').join(partsList);
        setSymbolValue(node, value);
    }

    @Override
    public void visit(ASTTypelessLiteral node) {
        super.visit(node);
        setSymbolValue(node, node.getValue());
    }

    @Override
    public void visit(ASTStringLiteral node) {
        super.visit(node);
        setSymbolValue(node, node.getValue());
    }

    @Override
    public void visit(ASTNullLiteral node) {
        super.visit(node);
        setSymbolValue(node, null);
    }

    @Override
    public void visit(ASTBooleanLiteral node) {
        super.visit(node);
        setSymbolValue(node, node.getValue());
    }

    @Override
    public void visit(ASTSignedDoubleLiteral node) {
        super.visit(node);
        setSymbolValue(node, node.getValue());
    }

    @Override
    public void visit(ASTSignedFloatLiteral node) {
        super.visit(node);
        setSymbolValue(node, node.getValue());
    }

    @Override
    public void visit(ASTSignedIntLiteral node) {
        super.visit(node);
        setSymbolValue(node, node.getValue());
    }

    @Override
    public void visit(ASTSignedLongLiteral node) {
        super.visit(node);
        setSymbolValue(node, node.getValue());
    }

    @Override
    public void visit(ASTListLiteral node) {
        super.visit(node);
        setSymbolValue(node, literalValue(node));
    }


    @Override
    protected void initialize_Configuration(ConfigurationSymbol confLangConfiguration, ASTConfiguration ast) {
        super.initialize_Configuration(confLangConfiguration, ast);

        Optional<ASTQualifiedName> superConfOpt = ast.getSuperConfOpt();
        if (superConfOpt.isPresent()) {
            ASTQualifiedName astmcQualifiedName = superConfOpt.get();
            String join = Joiner.on('.').join(astmcQualifiedName.getPartList());

            Optional<ConfigurationSymbol> superConfigurationSymbolOpt = globalScope.resolve(join, ConfigurationSymbol.KIND);
            if (superConfigurationSymbolOpt.isPresent()) {
                ConfigurationSymbol superConfiguration = superConfigurationSymbolOpt.get();
                confLangConfiguration.addSuperConfiguration(superConfiguration);

                Optional<ASTConfiguration> superConfigurationNodeOpt = superConfiguration.getConfigurationNode();
                if (superConfigurationNodeOpt.isPresent()) {
                    ast.addSuperConfiguration(superConfigurationNodeOpt.get());
                } else {
                    Log.warn(String.format("AST node of super configuration '%s' is not set.", superConfiguration.getName()));
                }
            }
        }
    }

    private void setSymbolValue(ASTSignedLiteral node, Object value) {
        if (configurationValueMap.containsKey(node)) {
            ASTConfigurationEntry entry = configurationValueMap.get(node);
            if (entry != null) {
                ConfigurationEntrySymbol symbol = entry.getConfigurationEntrySymbol();
                symbol.setValue(value);
            }

        } else if (nestedConfigurationValueMap.containsKey(node)) {
            ASTNestedConfigurationEntry entry = nestedConfigurationValueMap.get(node);
            if (entry != null) {
                NestedConfigurationEntrySymbol symbol = entry.getNestedConfigurationEntrySymbol();
                symbol.setValue(value);
            }
        }
    }
}