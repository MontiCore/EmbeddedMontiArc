package de.monticore.lang.monticar.borealis.runtime.language.ls;

import com.google.common.flogger.FluentLogger;

import java.util.Set;

public abstract class AbstractServerLauncher implements ServerLauncher {
    protected final FluentLogger logger;
    protected final Set<ServerLauncherContribution> contributions;

    protected AbstractServerLauncher(Set<ServerLauncherContribution> contributions) {
        this.logger = FluentLogger.forEnclosingClass();
        this.contributions = contributions;
    }

    @Override
    public void initialize(String[] arguments) throws Exception {
        this.logger.atInfo().log("Initializing contributions...");

        for (ServerLauncherContribution contribution : this.contributions) {
            contribution.onInitialize(arguments);
        }
    }

    @Override
    public void configure(String[] arguments) throws Exception {
        this.logger.atInfo().log("Configuring contributions...");

        for (ServerLauncherContribution contribution : this.contributions) {
            contribution.onConfigure(arguments);
        }
    }
}
