/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.lc.plugin.symboltable;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.monticar.sol.grammars.language._ast.ASTLanguage;
import de.monticore.lang.monticar.sol.grammars.language._ast.ASTLanguageCompilationUnit;
import de.monticore.lang.monticar.sol.grammars.language._symboltable.LanguageModelLoader;
import de.monticore.lang.monticar.sol.grammars.language._symboltable.LanguageSymbol;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.PluginContribution;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.mp.ModelPathService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm.NPMPackageService;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.configuration.LanguageClientConfiguration;
import org.apache.maven.plugin.Mojo;

import java.util.Collection;
import java.util.Optional;

@Singleton
public class LanguageSymbolTableImpl implements LanguageSymbolTable, PluginContribution {
    protected final NotificationService notifications;
    protected final NPMPackageService packages;
    protected final LanguageClientConfiguration configuration;
    protected final ModelPathService modelPath;
    protected final LanguageModelLoader loader;

    protected ASTLanguageCompilationUnit root;

    @Inject
    protected LanguageSymbolTableImpl(NotificationService notifications, NPMPackageService packages,
                                      LanguageClientConfiguration configuration, ModelPathService modelPath,
                                      LanguageModelLoader loader) {
        this.notifications = notifications;
        this.packages = packages;
        this.configuration = configuration;
        this.modelPath = modelPath;
        this.loader = loader;
    }

    @Override
    public Optional<ASTLanguageCompilationUnit> getRootNode() {
        return Optional.ofNullable(this.root);
    }

    @Override
    public Optional<LanguageSymbol> getRootSymbol() {
        return this.getRootNode()
                .map(ASTLanguageCompilationUnit::getLanguage)
                .flatMap(ASTLanguage::getLanguageSymbolOpt);
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
            Collection<ASTLanguageCompilationUnit> roots =
                    this.loader.loadModelsIntoScope(qualifiedName, modelPath);

            if (roots.iterator().hasNext()) this.root = roots.iterator().next();
            else throw new RuntimeException(String.format("Model '%s' could not be located.", qualifiedName));
        });
    }
}
