/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.PluginContribution;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import org.apache.maven.plugin.Mojo;

import java.util.HashSet;
import java.util.Set;

@Singleton
public class TemplateRegistryImpl implements TemplateRegistry, PluginContribution {
    protected final NotificationService notifications;
    protected final TemplateFactory factory;
    protected final Set<TemplateContribution> contributions;
    protected final Set<Template> templates;

    protected String templateRoot;
    protected String suffix;

    @Inject
    protected TemplateRegistryImpl(NotificationService notifications, TemplateFactory factory,
                                   Set<TemplateContribution> contributions) {
        this.notifications = notifications;
        this.factory = factory;
        this.contributions = contributions;
        this.templates = new HashSet<>();
        this.templateRoot = "";
        this.suffix = "";
    }

    @Override
    public Set<Template> getTemplates() {
        return this.templates;
    }

    @Override
    public void registerTemplate(String templatePath, String outputPath, Object ...arguments) {
        String realTemplatePath = String.format("%s/%s", this.templateRoot, templatePath);
        Template template = this.factory.create(realTemplatePath, outputPath, this.suffix, arguments);

        this.templates.add(template);
        this.notifications.info("Registered Template: %s.", template);
    }

    @Override
    public void setTemplateRoot(String templateRoot) {
        this.templateRoot = templateRoot.endsWith("/") ?
                templateRoot.substring(0, templateRoot.length() - 1) : templateRoot;

        this.notifications.debug("Template Root set to %s.", this.templateRoot);
    }

    @Override
    public String getTemplateRoot() {
        return this.templateRoot;
    }

    @Override
    public void setTopPatternSuffix(String suffix) {
        this.suffix = suffix;

        this.notifications.debug("Top Pattern Suffix set to %s.", this.suffix);
    }

    @Override
    public String getTopPatternSuffix() {
        return this.suffix;
    }

    @Override
    public int getPriority() {
        return 60;
    }

    @Override
    public void onPluginConfigure(Mojo plugin) {
        this.notifications.info("Registering Templates.");
        this.contributions.forEach(contribution -> contribution.registerTemplates(this));
    }
}
