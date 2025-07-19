/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.lc.plugin.generator.template;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.AbstractTemplateGeneratorPhase;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.TemplateRegistry;

@Singleton
public class TemplateGeneratorPhase extends AbstractTemplateGeneratorPhase {
    @Inject
    protected TemplateGeneratorPhase(NotificationService notifications,
                                     TemplateRegistry registry) {
        super(notifications, registry);
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
