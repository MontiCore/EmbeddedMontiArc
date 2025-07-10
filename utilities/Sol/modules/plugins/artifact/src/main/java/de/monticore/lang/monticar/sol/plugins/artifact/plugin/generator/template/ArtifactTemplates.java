/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.artifact.plugin.generator.template;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.grammars.artifact._ast.ASTArtifact;
import de.monticore.lang.monticar.sol.grammars.artifact._ast.ASTArtifactCompilationUnit;
import de.monticore.lang.monticar.sol.grammars.artifact._ast.ASTTool;
import de.monticore.lang.monticar.sol.grammars.artifact._symboltable.ArtifactSymbol;
import de.monticore.lang.monticar.sol.grammars.artifact._symboltable.ToolSymbol;
import de.monticore.lang.monticar.sol.plugins.artifact.plugin.symboltable.ArtifactSymbolTable;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm.NPMPackageService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm.SolPackage;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.TemplateContribution;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.TemplateRegistry;
import de.monticore.symboltable.Symbol;

import java.util.List;
import java.util.Optional;

@Singleton
public class ArtifactTemplates implements TemplateContribution {
    protected final ArtifactSymbolTable symbolTable;
    protected final NPMPackageService packages;

    @Inject
    protected ArtifactTemplates(ArtifactSymbolTable symbolTable, NPMPackageService packages) {
        this.symbolTable = symbolTable;
        this.packages = packages;
    }

    @Override
    public void registerTemplates(TemplateRegistry registry) {
        Symbol rootSymbol = this.getFirstArtifactSymbol()
                .orElseThrow(() -> new RuntimeException("There needs to be at least one root model."));
        SolPackage rootPackage = this.packages.getCurrentPackage()
                .orElseThrow(() -> new RuntimeException("Root Package could not be located."));
        List<ArtifactSymbol> artifacts = this.symbolTable.getArtifactSymbols();
        List<ToolSymbol> tools = this.symbolTable.getAllToolSymbols();
        String rootSymbolName = rootSymbol.getName();
        String rootSymbolNameLC = rootSymbolName.toLowerCase();

        registry.setTemplateRoot("templates/artifact/theia");
        registry.setTopPatternSuffix("-top");

        registry.registerTemplate(
                "node/artifact-backend-module.ftl",
                String.format("node/%s-backend-module.ts", rootSymbolNameLC),
                rootSymbolName, artifacts, tools
        );

        registry.registerTemplate(
                "node/env-variables-contribution.ftl",
                String.format("node/%s-env-variables-contribution.ts", rootSymbolNameLC),
                rootSymbolName, rootPackage, artifacts, tools
        );

        artifacts.forEach(artifact -> {
            String qualifiedFolder = artifact.getFullName().toLowerCase();

            registry.registerTemplate(
                    "node/artifact.ftl",
                    String.format("node/%s/artifact.ts", qualifiedFolder),
                    rootPackage, artifact
            );
        });

        tools.forEach(tool -> {
            String qualifiedFolder = tool.getFullName().toLowerCase();

            registry.registerTemplate(
                    "node/tool.ftl",
                    String.format("node/%s/tool.ts", qualifiedFolder),
                    rootPackage, tool
            );

            registry.registerTemplate(
                    "node/tool-factory.ftl",
                    String.format("node/%s/tool-factory.ts", qualifiedFolder),
                    tool
            );
        });
    }

    protected Optional<Symbol> getFirstArtifactSymbol() {
        List<ASTArtifactCompilationUnit> roots = this.symbolTable.getRootNodes();

        if (roots.isEmpty()) return Optional.empty();

        ASTArtifactCompilationUnit root = roots.get(0);

        if (root.getArtifactOpt().isPresent()) return root.getArtifactOpt().map(ASTArtifact::getArtifactSymbol);
        else if (root.getToolOpt().isPresent()) return root.getToolOpt().map(ASTTool::getToolSymbol);
        else return Optional.empty();
    }
}
