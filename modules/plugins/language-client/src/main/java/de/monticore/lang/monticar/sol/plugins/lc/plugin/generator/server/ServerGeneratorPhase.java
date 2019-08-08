/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.lc.plugin.generator.server;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.generating.GeneratorEngine;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.GeneratorPhase;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.configuration.LanguageClientConfiguration;
import org.apache.commons.io.FileUtils;

import java.io.File;
import java.nio.file.Paths;

@Singleton
public class ServerGeneratorPhase implements GeneratorPhase {
    protected final NotificationService notifications;
    protected final LanguageClientConfiguration configuration;

    @Inject
    protected ServerGeneratorPhase(NotificationService notifications, LanguageClientConfiguration configuration) {
        this.notifications = notifications;
        this.configuration = configuration;
    }

    @Override
    public String getLabel() {
        return "Language Client - Language Server Placement";
    }

    @Override
    public int getPriority() {
        return 48;
    }

    @Override
    public void generate(GeneratorEngine engine) throws Exception {
        String grammarName = this.configuration.getGrammarName();
        File serverArtifact = this.configuration.getServerArtifact();
        String outputPath = this.configuration.getOutputPath().getAbsolutePath();
        File targetArtifact = Paths.get(outputPath, "server", String.format("%s.jar", grammarName)).toFile();

        this.notifications.info("Copying Server from '%s' to '%s'.", serverArtifact, targetArtifact);
        FileUtils.copyFile(serverArtifact, targetArtifact);
    }
}
