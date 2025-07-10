/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.lc.plugin.generator.server;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.generating.GeneratorEngine;
import de.monticore.lang.monticar.sol.grammars.common._ast.CommonLiterals;
import de.monticore.lang.monticar.sol.grammars.language._symboltable.LanguageSymbol;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm.NPMPackageService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.path.PathResolver;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.GeneratorPhase;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.configuration.LanguageClientConfiguration;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.symboltable.LanguageSymbolTable;
import org.apache.commons.io.FileUtils;

import java.io.File;
import java.nio.file.Paths;

@Singleton
public class ServerGeneratorPhase implements GeneratorPhase {
    protected final NotificationService notifications;
    protected final LanguageClientConfiguration configuration;
    protected final NPMPackageService packages;
    protected final LanguageSymbolTable symbolTable;
    protected final PathResolver pathResolver;

    @Inject
    protected ServerGeneratorPhase(NotificationService notifications, LanguageClientConfiguration configuration,
                                   NPMPackageService packages, LanguageSymbolTable symbolTable,
                                   PathResolver pathResolver) {
        this.notifications = notifications;
        this.configuration = configuration;
        this.packages = packages;
        this.symbolTable = symbolTable;
        this.pathResolver = pathResolver;
    }

    @Override
    public String getLabel() {
        return "Language Client - Language Server Placement";
    }

    @Override
    public int getPriority() {
        return 48;
    }

    @Override
    public void generate(GeneratorEngine engine) throws Exception {
        String relativePath = this.packages.getCurrentPackage()
                .flatMap(solPackage -> solPackage.getDirectory("server")).orElse("server");
        String grammarName = this.configuration.getGrammarName();
        LanguageSymbol rootSymbol = this.symbolTable.getRootSymbol()
                .orElseThrow(() -> new Exception("Could not resolve root symbol."));
        String serverPath = rootSymbol.getServerPath()
                .orElseThrow(() -> new Exception("There is no server path specified in the model."));
        CommonLiterals origin = rootSymbol.getOrigin()
                .orElseThrow(() -> new Exception("There is no server origin specified in the model."));
        File serverArtifact = this.pathResolver.resolve(origin, serverPath).toFile();
        String outputPath = this.configuration.getOutputPath().getAbsolutePath();
        File targetArtifact = Paths.get(outputPath, relativePath, String.format("%s.jar", grammarName)).toFile();

        this.notifications.info("Copying Server from '%s' to '%s'.", serverArtifact, targetArtifact);
        FileUtils.copyFile(serverArtifact, targetArtifact);
    }
}
