/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.generate.template;

import de.monticore.generating.GeneratorEngine;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.GeneratorPhase;

import java.nio.file.Path;
import java.util.Set;

public abstract class AbstractTemplateGeneratorPhase implements GeneratorPhase {
    protected final NotificationService notifications;
    protected final Set<TemplateContribution> contributions;

    protected AbstractTemplateGeneratorPhase(NotificationService notifications,
                                             Set<TemplateContribution> contributions) {
        this.notifications = notifications;
        this.contributions = contributions;
    }

    @Override
    public void execute(GeneratorEngine engine) throws Exception {
        for (TemplateContribution contribution : this.contributions) {
            this.notifications.info("Generating from TemplateContribution: %s", contribution);

            Path outputPath = contribution.hasHandCodedPeer() ?
                    contribution.getTopPatternOutputFile() : contribution.getOutputPath();

            engine.generateNoA(contribution.getTemplatePath(), outputPath, contribution);
        }
    }
}
