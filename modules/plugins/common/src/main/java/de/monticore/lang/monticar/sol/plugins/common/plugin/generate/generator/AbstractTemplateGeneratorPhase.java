/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator;

import de.monticore.generating.GeneratorEngine;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.Template;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.TemplateRegistry;

import java.nio.file.Path;

public abstract class AbstractTemplateGeneratorPhase implements GeneratorPhase {
    protected final NotificationService notifications;
    protected final TemplateRegistry registry;

    protected AbstractTemplateGeneratorPhase(NotificationService notifications, TemplateRegistry registry) {
        this.notifications = notifications;
        this.registry = registry;
    }

    @Override
    public void generate(GeneratorEngine engine) {
        this.notifications.info("Generating from Templates.");

        for (Template template : this.registry.getTemplates()) {
            Path outputPath = template.hasHandwrittenPeer() ?
                    template.getTopPatternOutputPath() : template.getOutputPath();
            Object[] arguments = template.getArguments();

            this.notifications.info("Generating from Template %s to %s.", template, outputPath, arguments);
            engine.generateNoA(template.getTemplatePath(), outputPath, arguments);
        }
    }
}
