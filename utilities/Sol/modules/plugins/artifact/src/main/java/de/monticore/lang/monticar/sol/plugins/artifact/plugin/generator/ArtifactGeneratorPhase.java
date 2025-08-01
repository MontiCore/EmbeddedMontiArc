/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.artifact.plugin.generator;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.generating.GeneratorEngine;
import de.monticore.lang.monticar.sol.grammars.artifact._symboltable.ArtifactSymbol;
import de.monticore.lang.monticar.sol.grammars.artifact._symboltable.ToolSymbol;
import de.monticore.lang.monticar.sol.grammars.common._ast.CommonLiterals;
import de.monticore.lang.monticar.sol.grammars.common._symboltable.SymbolWithProjectPath;
import de.monticore.lang.monticar.sol.plugins.artifact.plugin.configuration.ArtifactConfiguration;
import de.monticore.lang.monticar.sol.plugins.artifact.plugin.symboltable.ArtifactSymbolTable;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm.NPMPackageService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm.SolPackage;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.path.PathResolver;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.GeneratorPhase;
import org.apache.commons.io.FileUtils;
import org.apache.maven.plugin.MojoExecutionException;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;

@Singleton
public class ArtifactGeneratorPhase implements GeneratorPhase {
    protected final NotificationService notifications;
    protected final ArtifactConfiguration configuration;
    protected final ArtifactSymbolTable symbolTable;
    protected final NPMPackageService packages;
    protected final PathResolver resolver;

    @Inject
    protected ArtifactGeneratorPhase(NotificationService notifications, ArtifactConfiguration configuration,
                                     ArtifactSymbolTable symbolTable, NPMPackageService packages,
                                     PathResolver resolver) {
        this.notifications = notifications;
        this.configuration = configuration;
        this.symbolTable = symbolTable;
        this.packages = packages;
        this.resolver = resolver;
    }

    @Override
    public String getLabel() {
        return "Artifact - Artifact Copying";
    }

    @Override
    public int getPriority() {
        return 45;
    }

    @Override
    public void generate(GeneratorEngine engine) throws MojoExecutionException, IOException {
        SolPackage rootPackage = this.packages.getCurrentPackage()
                .orElseThrow(() -> new MojoExecutionException("There is no root package."));
        String cwd = this.configuration.getMavenProject().getBasedir().toString();
        String relativePath = rootPackage.getDirectory("artifacts").orElse("sol/artifacts");
        Path artifactsDirectory = Paths.get(cwd, relativePath);

        this.notifications.info("Copying Artifacts.");
        this.copyTools(artifactsDirectory);
        this.copyArtifacts(artifactsDirectory);
    }

    protected void copyTools(Path artifactsDirectory) throws MojoExecutionException, IOException {
        List<ToolSymbol> tools = this.symbolTable.getToolSymbols();

        for (ToolSymbol tool : tools) {
            this.copyArtifact(artifactsDirectory, tool);
        }
    }

    protected void copyArtifacts(Path artifactsDirectory) throws MojoExecutionException, IOException {
        List<ArtifactSymbol> artifacts = this.symbolTable.getArtifactSymbols();

        for (ArtifactSymbol artifact : artifacts) {
            this.copyArtifact(artifactsDirectory, artifact);
        }
    }

    protected void copyArtifact(Path artifactsDirectory, SymbolWithProjectPath artifact) throws MojoExecutionException, IOException {
        CommonLiterals origin = artifact.getOrigin()
                .orElseThrow(() -> new MojoExecutionException("Origin is missing."));
        String relativePath = artifact.getPath()
                .orElseThrow(() -> new MojoExecutionException("Path is missing."));
        File destination = artifactsDirectory.resolve(artifact.getFullName().toLowerCase()).toFile();
        File source = this.resolver.resolve(origin, relativePath).toFile();

        if (source.isFile()) FileUtils.copyFileToDirectory(source, destination);
        else if (source.isDirectory()) FileUtils.copyDirectoryToDirectory(source, destination);
        else throw new MojoExecutionException("Artifact could not be located.");
    }
}
