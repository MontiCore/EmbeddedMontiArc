/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.environment.plugin.generator.ec.translator;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.generating.GeneratorEngine;
import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTInstruction;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.configuration.PluginConfiguration;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.prettyprint.IndentPrinter;
import de.se_rwth.commons.SourcePosition;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;

@Singleton
public class ECTranslatorImpl implements ECTranslator {
    protected final NotificationService notifications;
    protected final PluginConfiguration configuration;
    protected final IndentPrinter printer;

    protected String lastFilename;

    @Inject
    protected ECTranslatorImpl(NotificationService notifications, PluginConfiguration configuration) {
        this.notifications = notifications;
        this.configuration = configuration;
        this.printer = new IndentPrinter();
        this.lastFilename = "";
    }

    @Override
    public String translate(List<List<ASTInstruction>> partitions, GeneratorEngine engine) {
        this.printer.clearBuffer();

        for (List<ASTInstruction> partition : partitions) {
            ASTInstruction instruction = partition.get(0);
            String templatePath = String.format("templates/%s.ftl", instruction.getType());

            instruction.get_SourcePositionStartOpt().ifPresent(this::printFilename);
            this.printer.println(engine.generateNoA(templatePath, partition));
        }

        return this.printer.getContent();
    }

    protected void printFilename(SourcePosition position) {
        Path rootDirectory = this.configuration.getMavenProject().getBasedir().toPath();
        String filename = position.getFileName().orElse(rootDirectory.toString());
        String relativePath = rootDirectory.relativize(Paths.get(filename)).toString();

        if (this.lastFilename.equals(filename)) return;

        this.lastFilename = filename;

        this.printer.println();
        this.printer.println("#");
        this.printer.println(String.format("# %s", relativePath));
        this.printer.println("#");
    }
}
