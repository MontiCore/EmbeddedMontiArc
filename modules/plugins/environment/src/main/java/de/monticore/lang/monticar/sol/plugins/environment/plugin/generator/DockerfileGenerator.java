/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.sol.plugins.environment.plugin.generator;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.generating.GeneratorEngine;
import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTInstruction;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.GeneratorPhase;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.configuration.EnvironmentGenerateConfiguration;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.generator.ddf.collector.DDFCollector;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.generator.ddf.partitioner.DDFPartitioner;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.generator.ddf.sanitizer.DDFSanitizer;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.generator.ddf.translator.DDFTranslator;
import org.apache.commons.io.FileUtils;

import java.io.File;
import java.io.IOException;
import java.util.List;

@Singleton
public class DockerfileGenerator implements GeneratorPhase {
    protected final NotificationService notifications;
    protected final EnvironmentGenerateConfiguration configuration;
    protected final DDFCollector collector;
    protected final DDFSanitizer sanitizer;
    protected final DDFPartitioner partitioner;
    protected final DDFTranslator translator;

    @Inject
    protected DockerfileGenerator(NotificationService notifications, DDFCollector collector, DDFSanitizer sanitizer,
                                  DDFPartitioner partitioner, DDFTranslator translator,
                                  EnvironmentGenerateConfiguration configuration) {
        this.notifications = notifications;
        this.collector = collector;
        this.sanitizer = sanitizer;
        this.partitioner = partitioner;
        this.translator = translator;
        this.configuration = configuration;
    }

    @Override
    public String getLabel() {
        return "Environment Generator - Dockerfile Generation";
    }

    @Override
    public int getPriority() {
        return 100;
    }

    @Override
    public void generate(GeneratorEngine engine) throws IOException {
        List<ASTInstruction> unsanitizedInstructions = this.collector.collect();
        List<ASTInstruction> sanitizedInstructions = this.sanitizer.sanitize(unsanitizedInstructions);
        List<List<ASTInstruction>> partitions = this.partitioner.partition(sanitizedInstructions);
        String instructions = this.translator.translate(partitions, engine);
        File outputFile = new File(this.configuration.getOutputPath(), "Dockerfile");

        this.notifications.info("Generating Dockerfile.");
        FileUtils.writeStringToFile(outputFile, instructions, "UTF-8");
    }
}
