/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.environment.plugin.generate.generator;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.generating.GeneratorEngine;
import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTInstruction;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.GeneratorPhase;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.generate.generator.ddf.collector.DDFCollector;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.generate.generator.ddf.partitioner.DDFPartitioner;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.generate.generator.ddf.sanitizer.DDFSanitizer;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.generate.generator.ddf.translator.DDFTranslator;

import java.nio.file.Paths;
import java.util.List;

@Singleton
public class DockerfileGenerator implements GeneratorPhase {
    protected final NotificationService notifications;
    protected final DDFCollector collector;
    protected final DDFSanitizer sanitizer;
    protected final DDFPartitioner partitioner;
    protected final DDFTranslator translator;

    @Inject
    protected DockerfileGenerator(NotificationService notifications, DDFCollector collector, DDFSanitizer sanitizer,
                                  DDFPartitioner partitioner, DDFTranslator translator) {
        this.notifications = notifications;
        this.collector = collector;
        this.sanitizer = sanitizer;
        this.partitioner = partitioner;
        this.translator = translator;
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
    public void generate(GeneratorEngine engine) {
        List<ASTInstruction> unsanitizedInstructions = this.collector.collect();
        List<ASTInstruction> sanitizedInstructions = this.sanitizer.sanitize(unsanitizedInstructions);
        List<List<ASTInstruction>> partitions = this.partitioner.partition(sanitizedInstructions);
        String instructions = this.translator.translate(partitions, engine);

        this.notifications.info("Generating Dockerfile.");
        engine.generateNoA("templates/Dockerfile.ftl", Paths.get("Dockerfile"), instructions);
    }
}
