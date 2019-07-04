/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.ls.plugin.generator;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.template.AbstractTemplateGeneratorPhase;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.template.TemplateContribution;

import java.util.Set;

@Singleton
public class LanguageServerTemplateGeneratorPhase extends AbstractTemplateGeneratorPhase {
    @Inject
    protected LanguageServerTemplateGeneratorPhase(NotificationService notifications,
                                                   Set<TemplateContribution> contributions) {
        super(notifications, contributions);
    }

    @Override
    public String getLabel() {
        return "Language Server Generator - Template Generation";
    }

    @Override
    public int getPriority() {
        return 50;
    }
}
