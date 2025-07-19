/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.variable;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.Template;

import java.util.Set;

@Singleton
public class TemplateVariableServiceImpl implements TemplateVariableService {
    protected final NotificationService notifications;
    protected final Set<TemplateVariable> variables;

    @Inject
    protected TemplateVariableServiceImpl(NotificationService notifications, Set<TemplateVariable> variables) {
        this.notifications = notifications;
        this.variables = variables;
    }

    @Override
    public String resolve(String text, Template template) {
        String resolvedText = text;

        for (TemplateVariable variable : this.variables) {
            String regex = String.format("\\$\\{%s\\}", variable.getIdentifier());

            resolvedText = resolvedText.replaceAll(regex, variable.resolve(template));
        }

        this.notifications.debug("%s resolved to %s.", text, resolvedText);
        return resolvedText;
    }
}
