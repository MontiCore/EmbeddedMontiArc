/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.generating.GeneratorEngine;
import de.monticore.generating.GeneratorSetup;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.PluginContribution;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import org.apache.maven.plugin.Mojo;

import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

@Singleton
public class GeneratorImpl implements Generator, PluginContribution {
    protected final NotificationService notifications;
    protected final List<GeneratorPhase> phases;
    protected final Set<GlexContribution> glexContributions;
    protected final Set<GeneratorSetupContribution> setupContributions;
    protected final GeneratorEngine engine;
    protected final GeneratorSetup setup;

    @Inject
    protected GeneratorImpl(
            NotificationService notifications, Set<GeneratorPhase> phases, Set<GlexContribution> glexContributions,
            Set<GeneratorSetupContribution> setupContributions
    ) {
        this.notifications = notifications;
        this.setup = new GeneratorSetup();
        this.engine = new GeneratorEngine(this.setup);
        this.phases = this.sortPhases(phases);
        this.glexContributions = glexContributions;
        this.setupContributions = setupContributions;
    }

    protected List<GeneratorPhase> sortPhases(Set<GeneratorPhase> phases) {
        Comparator<GeneratorPhase> comparator = Comparator.comparingInt(GeneratorPhase::getPriority);
        List<GeneratorPhase> sortedPhases = phases.stream().sorted(comparator).collect(Collectors.toList());

        Collections.reverse(sortedPhases);

        return sortedPhases;
    }

    @Override
    public int getPriority() {
        return 40;
    }

    @Override
    public void configure() {
        this.notifications.info("Configuring Generator.");

        for (GeneratorSetupContribution contribution : this.setupContributions) {
            contribution.setup(this.setup);
        }

        for (GlexContribution contribution : this.glexContributions) {
            contribution.defineGlobalVars(this.setup.getGlex());
        }
    }

    @Override
    public void canGenerate() throws Exception {
        this.notifications.info("Checking Generator Prerequisites.");

        for (GeneratorPhase phase : this.phases) {
            phase.canExecute();
        }
    }

    @Override
    public void generate() throws Exception {
        int n = this.phases.size();

        this.notifications.info("Executing %d-Phase Generator.", n);

        for (int i = 0; i < n; i++) {
            GeneratorPhase phase = this.phases.get(i);

            this.notifications.info("[Phase %d/%d]: %s", i+1, n, phase.getLabel());

            if (phase.shouldExecute()) phase.execute(this.engine);
        }
    }

    @Override
    public void onPluginConfigure(Mojo plugin) {
        this.configure();
    }

    @Override
    public void onPluginExecute(Mojo plugin) throws Exception {
        this.canGenerate();
        this.generate();
    }
}
