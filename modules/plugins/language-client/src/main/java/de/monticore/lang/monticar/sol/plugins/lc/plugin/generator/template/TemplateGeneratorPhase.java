/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.lc.plugin.generator.template;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.template.AbstractTemplateGeneratorPhase;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.template.TemplateContribution;

import java.util.Set;

@Singleton
public class TemplateGeneratorPhase extends AbstractTemplateGeneratorPhase {
    @Inject
    protected TemplateGeneratorPhase(NotificationService notifications,
                                     Set<TemplateContribution> contributions) {
        super(notifications, contributions);
    }

    @Override
    public String getLabel() {
        return "Language Client - Template Generation";
    }

    @Override
    public int getPriority() {
        return 50;
    }
}
