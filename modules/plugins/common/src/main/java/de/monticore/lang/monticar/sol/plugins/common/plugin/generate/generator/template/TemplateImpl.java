/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template;

import com.google.inject.assistedinject.Assisted;
import com.google.inject.assistedinject.AssistedInject;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.hc.HandCodeRegistry;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.variable.TemplateVariableService;
import org.apache.commons.io.FilenameUtils;
import org.apache.commons.lang3.ArrayUtils;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.function.Predicate;

public class TemplateImpl implements Template {
    protected final String templatePath;
    protected final String outputPath;
    protected final String suffix;
    protected final Object[] arguments;
    protected final TemplateVariableService resolver;
    protected final HandCodeRegistry peerRegistry;

    @AssistedInject
    public TemplateImpl(@Assisted("templatePath") String templatePath, @Assisted("outputPath") String outputPath,
                        @Assisted("suffix") String suffix, @Assisted("arguments") Object[] arguments,
                        TemplateVariableService resolver, HandCodeRegistry peerRegistry) {
        this.templatePath = templatePath;
        this.outputPath = outputPath;
        this.arguments = ArrayUtils.add(arguments, 0, this);
        this.resolver = resolver;
        this.peerRegistry = peerRegistry;
        this.suffix = suffix;
    }

    @AssistedInject
    public TemplateImpl(@Assisted("templatePath") String templatePath, @Assisted("outputPath") String outputPath,
                        @Assisted("arguments") Object[] arguments,TemplateVariableService resolver,
                        HandCodeRegistry peerRegistry) {
        this.templatePath = templatePath;
        this.outputPath = outputPath;
        this.arguments = ArrayUtils.add(arguments, 0, this);
        this.resolver = resolver;
        this.peerRegistry = peerRegistry;
        this.suffix = "";
    }

    @Override
    public String getTemplatePath() {
        return this.templatePath;
    }

    @Override
    public Path getOutputPath() {
        return Paths.get(this.resolver.resolve(this.outputPath, this));
    }

    @Override
    public Path getTopPatternOutputPath() {
        String outputPath = this.getOutputPath().toString();
        String extension = FilenameUtils.getExtension(outputPath);
        String pathWithoutExtension = FilenameUtils.removeExtension(outputPath);

        return Paths.get(String.format("%s%s.%s", pathWithoutExtension, this.suffix, extension));
    }

    @Override
    public boolean hasHandwrittenPeer() {
        Predicate<Path> predicate = path -> path.equals(this.getOutputPath());

        return this.peerRegistry.getHandCodes().stream().anyMatch(predicate);
    }

    @Override
    public Object[] getArguments() {
        return this.arguments;
    }

    @Override
    public String toString() {
        return String.format("{ Template: %s, HandwrittenPeer: %b }", this.templatePath, this.hasHandwrittenPeer());
    }

    @Override
    public boolean equals(Object o) {
        return o instanceof Template && this.equals((Template) o);
    }

    protected boolean equals(Template peer) {
        return this.getTemplatePath().equals(peer.getTemplatePath())
                && this.getOutputPath().equals(peer.getOutputPath())
                && this.getTopPatternOutputPath().equals(peer.getTopPatternOutputPath())
                && this.hasHandwrittenPeer() == peer.hasHandwrittenPeer();
    }
}
