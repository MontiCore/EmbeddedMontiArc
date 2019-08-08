/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.lc.plugin.validator;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.grammars.language._cocos.LanguageCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.language._parser.LanguageParser;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.PluginContribution;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.configuration.LanguageClientConfiguration;
import de.se_rwth.commons.logging.Log;
import org.apache.maven.plugin.Mojo;
import org.apache.maven.plugin.MojoExecutionException;

import java.io.File;
import java.util.List;

@Singleton
public class LDValidatorImpl implements LDValidator, PluginContribution {
    protected final NotificationService notifications;
    protected final LanguageClientConfiguration configuration;
    protected final LanguageCoCoChecker checker;
    protected final LanguageParser parser;

    @Inject
    protected LDValidatorImpl(NotificationService notifications, LanguageClientConfiguration configuration,
                              LanguageCoCoChecker checker) {
        this.notifications = notifications;
        this.configuration = configuration;
        this.checker = checker;
        this.parser = new LanguageParser();
    }

    @Override
    public void validate() throws Exception {
        List<File> models = this.configuration.getModels();

        Log.enableFailQuick(false);
        this.notifications.info("Validating Models.");

        for (File model : models) {
            this.notifications.info("Validating '%s'.", model);
            this.parser.parse(model.getPath()).ifPresent(this.checker::checkAll);
        }

        if (Log.getFindings().size() > 0) throw new MojoExecutionException("There are erroneous models.");
    }

    @Override
    public int getPriority() {
        return 50;
    }

    @Override
    public void onPluginConfigure(Mojo plugin) throws Exception {
        this.validate();
    }
}
