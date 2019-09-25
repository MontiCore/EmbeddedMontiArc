/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.lc.plugin.configuration;

import com.google.common.base.Preconditions;
import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.configuration.AbstractGeneratePluginConfiguration;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.LanguageClientPlugin;
import org.apache.commons.io.FileUtils;
import org.apache.commons.io.filefilter.NameFileFilter;
import org.apache.commons.io.filefilter.TrueFileFilter;
import org.apache.maven.project.MavenProject;

import java.io.File;
import java.io.IOException;
import java.nio.file.Paths;
import java.util.Collection;
import java.util.List;
import java.util.function.Predicate;

@Singleton
public class LanguageClientConfigurationImpl extends AbstractGeneratePluginConfiguration
        implements LanguageClientConfiguration {
    private final LanguageClientPlugin plugin;

    @Inject
    protected LanguageClientConfigurationImpl(LanguageClientPlugin plugin) {
        super(plugin);

        this.plugin = plugin;
    }

    @Override
    protected File getDefaultHandCodedPath() {
        return this.resolveFromBaseDirectory("src");
    }

    @Override
    public String getGrammarQualifiedName() {
        Preconditions.checkNotNull(this.plugin);

        return this.plugin.getGrammar();
    }

    @Override
    public String getGrammarName() {
        String qualifiedName = this.getGrammarQualifiedName();
        String[] qualifiedParts = qualifiedName.split("\\.");

        return qualifiedParts[qualifiedParts.length - 1];
    }

    @Override
    public String getFileExtension() {
        Preconditions.checkNotNull(this.plugin);

        String extension = this.plugin.getExtension();

        return extension.startsWith(".") ? extension.substring(1) : extension;
    }

    @Override
    public List<String> getExcludedKeywords() {
        Preconditions.checkNotNull(this.plugin);

        return this.plugin.getExcludedKeywords();
    }

    @Override
    public File getSourceCodeOutputPath() {
        return new File(this.getOutputPath(), "src-gen");
    }

    @Override
    public File getTokensArtifact() throws IOException {
        Preconditions.checkNotNull(this.plugin);

        String fileName = String.format("%sAntlr.tokens", this.getGrammarName());
        IOException exception = new IOException(String.format("Tokens File %s could not be located.", fileName));
        MavenProject grammarModule = this.getMavenProjectFromArtifactId(this.plugin.getGrammarModule(), exception);
        NameFileFilter filter = new NameFileFilter(fileName);
        File rootDir = new File(grammarModule.getBuild().getDirectory());
        Collection<File> tokensFiles = FileUtils.listFiles(rootDir, filter, TrueFileFilter.INSTANCE);

        if (tokensFiles.size() > 0) return tokensFiles.iterator().next();
        else throw exception;
    }

    @Override
    public File getServerArtifact() throws IOException {
        Preconditions.checkNotNull(this.plugin);

        String[] identifiers = this.plugin.getServerArtifact().split(":");
        String serverArtifact = identifiers[1];
        IOException exception = new IOException(String.format("Server %s could not be located.", serverArtifact));
        MavenProject serverModule = this.getMavenProjectFromArtifactId(identifiers[0], exception);

        return Paths.get(serverModule.getBuild().getDirectory(), serverArtifact).toFile();
    }

    @Override
    public String getRootModel() {
        Preconditions.checkNotNull(this.plugin);

        return this.plugin.getRootModel();
    }

    protected MavenProject getMavenProjectFromArtifactId(String artifactId, IOException exception) throws IOException {
        List<MavenProject> projects = this.plugin.getMavenProject().getParent().getCollectedProjects();
        Predicate<MavenProject> predicate = project -> project.getArtifactId().equals(artifactId);

        return projects.stream().filter(predicate).findFirst().orElseThrow(() -> exception);
    }
}
