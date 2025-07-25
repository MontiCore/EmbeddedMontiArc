/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.ide.plugin.symboltable;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.monticar.sol.grammars.ide._ast.ASTIDE;
import de.monticore.lang.monticar.sol.grammars.ide._ast.ASTIDECompilationUnit;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.IDEModelLoader;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.IDESymbol;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.PluginContribution;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.mp.ModelPathService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm.NPMPackageService;
import de.monticore.lang.monticar.sol.plugins.ide.plugin.configuration.IDEConfiguration;
import org.apache.maven.plugin.Mojo;

import java.util.Collection;
import java.util.Optional;

@Singleton
public class IDESymbolTableImpl implements IDESymbolTable, PluginContribution {
    protected final NotificationService notifications;
    protected final NPMPackageService packages;
    protected final IDEConfiguration configuration;
    protected final ModelPathService modelPath;
    protected final IDEModelLoader loader;

    protected ASTIDECompilationUnit root;

    @Inject
    protected IDESymbolTableImpl(NotificationService notifications, NPMPackageService packages,
                                 IDEConfiguration configuration, ModelPathService modelPath, IDEModelLoader loader) {
        this.notifications = notifications;
        this.packages = packages;
        this.configuration = configuration;
        this.modelPath = modelPath;
        this.loader = loader;
    }

    @Override
    public Optional<ASTIDECompilationUnit> getRootNode() {
        return Optional.ofNullable(this.root);
    }

    @Override
    public Optional<IDESymbol> getRootSymbol() {
        return Optional.ofNullable(this.root).flatMap(ASTIDECompilationUnit::getIDEOpt).map(ASTIDE::getIDESymbol);
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
            Collection<ASTIDECompilationUnit> roots =
                    this.loader.loadModelsIntoScope(qualifiedName, modelPath);

            if (roots.iterator().hasNext()) this.root = roots.iterator().next();
            else throw new RuntimeException(String.format("Model '%s' could not be located.", qualifiedName));
        });
    }
}
