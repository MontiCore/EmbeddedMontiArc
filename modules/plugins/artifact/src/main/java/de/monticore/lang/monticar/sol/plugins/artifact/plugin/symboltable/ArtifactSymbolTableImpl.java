/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.artifact.plugin.symboltable;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.monticar.sol.grammars.artifact._ast.ASTArtifact;
import de.monticore.lang.monticar.sol.grammars.artifact._ast.ASTArtifactCompilationUnit;
import de.monticore.lang.monticar.sol.grammars.artifact._ast.ASTTool;
import de.monticore.lang.monticar.sol.grammars.artifact._symboltable.ArtifactModelLoader;
import de.monticore.lang.monticar.sol.grammars.artifact._symboltable.ArtifactSymbol;
import de.monticore.lang.monticar.sol.grammars.artifact._symboltable.ToolSymbol;
import de.monticore.lang.monticar.sol.plugins.artifact.plugin.configuration.ArtifactConfiguration;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.PluginContribution;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.mp.ModelPathService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm.NPMPackageService;
import org.apache.maven.plugin.Mojo;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

@Singleton
public class ArtifactSymbolTableImpl implements ArtifactSymbolTable, PluginContribution {
    protected final NotificationService notifications;
    protected final NPMPackageService packages;
    protected final ArtifactConfiguration configuration;
    protected final ModelPathService modelPath;
    protected final ArtifactModelLoader loader;

    protected List<ASTArtifactCompilationUnit> roots;

    @Inject
    protected ArtifactSymbolTableImpl(NotificationService notifications, NPMPackageService packages,
                                      ArtifactConfiguration configuration, ModelPathService modelPath,
                                      ArtifactModelLoader loader) {
        this.notifications = notifications;
        this.packages = packages;
        this.configuration = configuration;
        this.modelPath = modelPath;
        this.loader = loader;
        this.roots = new ArrayList<>();
    }

    @Override
    public List<ASTArtifactCompilationUnit> getRootNodes() {
        return this.roots;
    }

    @Override
    public List<ArtifactSymbol> getArtifactSymbols() {
        return this.roots.stream()
                .map(node -> node.getArtifactOpt().flatMap(ASTArtifact::getArtifactSymbolOpt))
                .filter(Optional::isPresent)
                .map(Optional::get)
                .collect(Collectors.toList());
    }

    @Override
    public List<ToolSymbol> getAllToolSymbols() {
        return this.roots.stream()
                .map(node -> node.getToolOpt().flatMap(ASTTool::getToolSymbolOpt))
                .filter(Optional::isPresent)
                .map(Optional::get)
                .collect(Collectors.toList());
    }

    @Override
    public List<ToolSymbol> getToolSymbols() {
        return this.getAllToolSymbols().stream()
                .filter(tool -> !tool.isVirtual())
                .collect(Collectors.toList());
    }

    @Override
    public List<ToolSymbol> getVirtualToolSymbols() {
        return this.getAllToolSymbols().stream()
                .filter(ToolSymbol::isVirtual)
                .collect(Collectors.toList());
    }

    @Override
    public int getPriority() {
        return 70;
    }

    @Override
    public void onPluginConfigure(Mojo plugin) {
        this.packages.getCurrentPackage().ifPresent(rootPackage -> {
            this.notifications.info("Building Symbol Table.");

            ModelPath modelPath = this.modelPath.resolve(rootPackage);
            List<String> qualifiedNames = this.configuration.getRootModels();

            this.loadModelsIntoScope(qualifiedNames, modelPath);
        });
    }

    protected void loadModelsIntoScope(List<String> qualifiedNames, ModelPath modelPath) {
        qualifiedNames.forEach(qualifiedName -> {
            Collection<ASTArtifactCompilationUnit> roots =
                    this.loader.loadModelsIntoScope(qualifiedName, modelPath);

            if (roots.iterator().hasNext()) this.roots.add(roots.iterator().next());
            else throw new RuntimeException(String.format("Model '%s' could not be located.", qualifiedName));
        });
    }
}
