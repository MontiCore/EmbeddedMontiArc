/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.environment.plugin.symboltable;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTEnvironmentCompilationUnit;
import de.monticore.lang.monticar.sol.grammars.environment._symboltable.EnvironmentModelLoader;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.PluginContribution;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.mp.ModelPathService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm.NPMPackageService;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.configuration.EnvironmentGenerateConfiguration;
import org.apache.maven.plugin.Mojo;

import java.util.Collection;
import java.util.Optional;

@Singleton
public class EnvironmentSymbolTableImpl implements EnvironmentSymbolTable, PluginContribution {
    protected final NotificationService notifications;
    protected final NPMPackageService packages;
    protected final EnvironmentGenerateConfiguration configuration;
    protected final ModelPathService modelPath;
    protected final EnvironmentModelLoader loader;

    protected ASTEnvironmentCompilationUnit root;

    @Inject
    protected EnvironmentSymbolTableImpl(NotificationService notifications, NPMPackageService packages,
                                         EnvironmentGenerateConfiguration configuration, ModelPathService modelPath,
                                         EnvironmentModelLoader loader) {
        this.notifications = notifications;
        this.packages = packages;
        this.configuration = configuration;
        this.modelPath = modelPath;
        this.loader = loader;
    }

    @Override
    public Optional<ASTEnvironmentCompilationUnit> getRootNode() {
        return Optional.ofNullable(this.root);
    }

    @Override
    public int getPriority() {
        return 70;
    }

    @Override
    public void onPluginConfigure(Mojo plugin) {
        this.packages.getCurrentPackage().ifPresent(rootPackage -> {
            this.notifications.info("Building Symbol Table.");

            String qualifiedName = this.configuration.getRootModel();
            ModelPath modelPath = this.modelPath.resolve(rootPackage);
            Collection<ASTEnvironmentCompilationUnit> roots =
                    this.loader.loadModelsIntoScope(qualifiedName, modelPath);

            if (roots.iterator().hasNext()) this.root = roots.iterator().next();
            else throw new RuntimeException(String.format("Model '%s' could not be located.", qualifiedName));
        });
    }
}
