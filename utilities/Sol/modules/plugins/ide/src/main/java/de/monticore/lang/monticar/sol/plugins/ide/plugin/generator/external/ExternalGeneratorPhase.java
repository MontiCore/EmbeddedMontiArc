/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.ide.plugin.generator.external;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.generating.GeneratorEngine;
import de.monticore.generating.GeneratorSetup;
import de.monticore.lang.monticar.sol.grammars.common._ast.CommonLiterals;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.IDESymbol;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm.NPMPackageService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm.SolPackage;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.path.PathResolver;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.GeneratorPhase;
import de.monticore.lang.monticar.sol.plugins.ide.plugin.configuration.IDEConfiguration;
import de.monticore.lang.monticar.sol.plugins.ide.plugin.generator.external.resolver.SolExtensionsResolver;
import de.monticore.lang.monticar.sol.plugins.ide.plugin.symboltable.IDESymbolTable;
import org.apache.commons.io.FileUtils;
import org.apache.maven.plugin.MojoExecutionException;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import java.util.Set;

@Singleton
public class ExternalGeneratorPhase implements GeneratorPhase {
    protected final NotificationService notifications;
    protected final NPMPackageService packages;
    protected final IDESymbolTable symbolTable;
    protected final SolExtensionsResolver resolver;
    protected final PathResolver pathResolver;
    protected final GeneratorSetup setup;
    protected final IDEConfiguration configuration;

    @Inject
    protected ExternalGeneratorPhase(NotificationService notifications, NPMPackageService packages,
                                     IDESymbolTable symbolTable, SolExtensionsResolver resolver,
                                     PathResolver pathResolver, IDEConfiguration configuration, GeneratorSetup setup) {
        this.notifications = notifications;
        this.packages = packages;
        this.symbolTable = symbolTable;
        this.resolver = resolver;
        this.pathResolver = pathResolver;
        this.configuration = configuration;
        this.setup = setup;
    }

    @Override
    public String getLabel() {
        return "IDE - External Generation";
    }

    @Override
    public int getPriority() {
        return 30;
    }

    @Override
    public void generate(GeneratorEngine engine) throws Exception {
        IDESymbol rootSymbol = this.symbolTable.getRootSymbol()
                .orElseThrow(() -> new MojoExecutionException("Root Symbol could not be located."));
        String cwd = this.configuration.getMavenProject().getBasedir().toString();
        File outputDirectory = Paths.get(cwd,"..", "external").toFile();

        this.generateFiles(engine, rootSymbol, outputDirectory);
        this.generateBuildFolder(rootSymbol, outputDirectory);
    }

    protected void generateFiles(GeneratorEngine engine, IDESymbol rootSymbol, File outputDirectory) throws MojoExecutionException {
        SolPackage rootPackage = this.packages.getCurrentPackage()
                .orElseThrow(() -> new MojoExecutionException("Root Package could not be located."));

        this.setup.setOutputDirectory(outputDirectory);
        this.generatePackage(engine, rootPackage);
        this.generateTypeScriptConfig(engine);
        this.generateWebPackConfiguration(engine);
        this.generateElectronBuilderConfiguration(engine, rootSymbol, rootPackage);
        this.generateEntry(engine, rootSymbol, rootPackage, "main");
        this.generateEntry(engine, rootSymbol, rootPackage, "renderer");
    }

    protected void generateBuildFolder(IDESymbol rootSymbol, File outputDirectory) throws IOException, MojoExecutionException {
        String errorMessage = "Root Symbol is missing a build folder.";
        CommonLiterals origin = rootSymbol.getBuildOrigin()
                .orElseThrow(() -> new MojoExecutionException(errorMessage));
        String relativePath = rootSymbol.getBuildPath()
                .orElseThrow(() -> new MojoExecutionException(errorMessage));
        File source = this.pathResolver.resolve(origin, relativePath).toFile();
        File destination = new File(outputDirectory, "build");

        FileUtils.copyDirectory(source, destination);
    }

    protected void generatePackage(GeneratorEngine engine, SolPackage rootPackage) {
        Set<SolPackage> allPackages = this.resolver.resolveAllPackagesWithExtensions(rootPackage);
        Path outputPath = Paths.get("package.json");

        this.notifications.info("Generating 'package.json' to '%s'.", outputPath);
        engine.generateNoA("templates/ide/external/package.ftl", outputPath, rootPackage, allPackages);
    }

    protected void generateTypeScriptConfig(GeneratorEngine engine) {
        Path outputPath = Paths.get("tsconfig.json");

        this.notifications.info("Generating 'tsconfig.json' to '%s'", outputPath);
        engine.generateNoA("templates/ide/external/tsconfig.ftl", outputPath);
    }

    protected void generateWebPackConfiguration(GeneratorEngine engine) {
        Path outputPath = Paths.get("configs/webpack.config.js");

        this.notifications.info("Generating 'webpack.config.js' to '%s'.", outputPath);
        engine.generateNoA("templates/ide/external/configs/webpack-config.ftl", outputPath);
    }

    protected void generateElectronBuilderConfiguration(GeneratorEngine engine, IDESymbol ide, SolPackage rootPackage) {
        Path outputPath = Paths.get("configs/electron-builder.yml");

        this.notifications.info("Generating 'electron-builder.yml' to '%s'.", outputPath);
        engine.generateNoA("templates/ide/external/configs/electron-builder.ftl", outputPath, ide, rootPackage);
    }

    protected void generateEntry(GeneratorEngine engine, IDESymbol symbol, SolPackage rootPackage, String folder) {
        Path outputPath = Paths.get(String.format("src-gen/%s/index.ts", folder));
        List<String> extensions = this.resolver.resolveExtensions(rootPackage, folder);
        Object[] arguments = folder.equals("main") ? new Object[] { symbol, extensions } : new Object[] { extensions };

        this.notifications.info("Generating 'index.ts' to '%s'.", outputPath);
        engine.generateNoA(String.format("templates/ide/external/src/%s/index.ftl", folder), outputPath, arguments);
    }
}
