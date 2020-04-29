/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.ide.plugin.generator.internal;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.generating.GeneratorEngine;
import de.monticore.generating.GeneratorSetup;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.IDESymbol;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm.NPMPackageService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm.SolPackage;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm.TheiaPackage;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.GeneratorPhase;
import de.monticore.lang.monticar.sol.plugins.ide.plugin.configuration.IDEConfiguration;
import de.monticore.lang.monticar.sol.plugins.ide.plugin.symboltable.IDESymbolTable;
import org.apache.maven.plugin.MojoExecutionException;
import org.json.JSONObject;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.HashSet;
import java.util.Set;

@Singleton
public class InternalGeneratorPhase implements GeneratorPhase {
    protected final NotificationService notifications;
    protected final IDEConfiguration configuration;
    protected final IDESymbolTable symbolTable;
    protected final NPMPackageService packages;
    protected final GeneratorSetup setup;

    @Inject
    protected InternalGeneratorPhase(NotificationService notifications, IDESymbolTable symbolTable,
                                     NPMPackageService packages, IDEConfiguration configuration, GeneratorSetup setup) {
        this.notifications = notifications;
        this.symbolTable = symbolTable;
        this.packages = packages;
        this.configuration = configuration;
        this.setup = setup;
    }

    @Override
    public String getLabel() {
        return "IDE - Internal Generation";
    }

    @Override
    public int getPriority() {
        return 40;
    }

    @Override
    public void generate(GeneratorEngine engine) throws Exception {
        IDESymbol rootSymbol = this.symbolTable.getRootSymbol()
                .orElseThrow(() -> new MojoExecutionException("Root Symbol should be present."));
        String cwd = this.configuration.getMavenProject().getBasedir().toString();

        this.setup.setOutputDirectory(Paths.get(cwd, "..", "internal").toFile());
        this.generatePackage(engine, rootSymbol);
        this.generateEnvironment(engine, rootSymbol);
    }

    protected void generatePackage(GeneratorEngine engine, IDESymbol rootSymbol) throws MojoExecutionException {
        TheiaPackage rootPackage = this.packages.getCurrentPackage()
                .flatMap(SolPackage::getAsTheiaPackage)
                .orElseThrow(() -> new MojoExecutionException("Root Package should be a Theia extension."));
        Set<TheiaPackage> allPackages = new HashSet<>(rootPackage.getAllTheiaDependencies());
        Path outputPath = Paths.get("package.json");
        JSONObject resolutions = rootPackage.<JSONObject>query("/resolutions").orElse(new JSONObject());

        allPackages.add(rootPackage);
        this.notifications.info("Generating 'package.json' to '%s'.", outputPath);
        engine.generateNoA("templates/ide/internal/package.ftl",
                outputPath, rootSymbol, rootPackage, allPackages, resolutions);
    }

    protected void generateEnvironment(GeneratorEngine engine, IDESymbol rootSymbol) {
        String modelsPath = String.format("%s.ec", rootSymbol.getFullName().replace(".", "/"));
        Path outputPath = Paths.get("sol", "models", modelsPath);

        this.notifications.info("Generating '%s.ec' to '%s'.", rootSymbol.getName(), outputPath);
        engine.generateNoA("templates/ide/internal/models/IDE.ftl", outputPath, rootSymbol);
    }
}
