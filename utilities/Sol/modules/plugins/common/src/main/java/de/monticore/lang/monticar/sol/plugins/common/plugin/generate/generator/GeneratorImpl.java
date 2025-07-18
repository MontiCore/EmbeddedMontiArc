/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.generating.GeneratorEngine;
import de.monticore.generating.GeneratorSetup;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.PluginContribution;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import org.apache.maven.plugin.Mojo;

import java.util.List;
import java.util.Set;

@Singleton
public class GeneratorImpl implements Generator, PluginContribution {
    protected final NotificationService notifications;
    protected final List<GeneratorPhase> phases;
    protected final Set<GlexContribution> glexContributions;
    protected final Set<GeneratorSetupContribution> setupContributions;
    protected final GeneratorEngine engine;
    protected final GeneratorSetup setup;

    @Inject
    protected GeneratorImpl(NotificationService notifications, GeneratorEngine engine, GeneratorSetup setup,
                            List<GeneratorPhase> phases, Set<GlexContribution> glexContributions,
                            Set<GeneratorSetupContribution> setupContributions) {
        this.notifications = notifications;
        this.setup = setup;
        this.engine = engine;
        this.phases = phases;
        this.glexContributions = glexContributions;
        this.setupContributions = setupContributions;
    }

    @Override
    public int getPriority() {
        return 40;
    }

    @Override
    public void configure() throws Exception {
        this.notifications.info("Configuring Generator.");

        for (GeneratorSetupContribution contribution : this.setupContributions) {
            contribution.setup(this.setup);
        }

        for (GlexContribution contribution : this.glexContributions) {
            contribution.defineGlobalVars(this.setup.getGlex());
        }

        for (GeneratorPhase phase : this.phases) {
            phase.configure();
        }
    }

    @Override
    public void generate() throws Exception {
        int size = this.phases.size();

        this.notifications.info("Executing %d-Phase Generator.", size);

        for (GeneratorPhase phase : this.phases) {
            int index = 1 + this.phases.indexOf(phase);

            this.notifications.info("[Phase %d/%d]: %s", index, size, phase.getLabel());

            if (phase.shouldGenerate()) phase.generate(this.engine);
        }
    }

    @Override
    public void shutdown() throws Exception {
        this.notifications.info("Shutting down Generator.");

        for (GeneratorPhase phase : this.phases) {
            phase.shutdown();
        }
    }

    @Override
    public void onPluginConfigure(Mojo plugin) throws Exception {
        this.configure();
    }

    @Override
    public void onPluginExecute(Mojo plugin) throws Exception {
        this.generate();
    }

    @Override
    public void onPluginShutdown(Mojo plugin) throws Exception {
        this.shutdown();
    }
}
