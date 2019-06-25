package de.monticore.lang.monticar.borealis.runtime.language.ls;

import java.util.Set;

public abstract class AbstractServerLauncher implements ServerLauncher {
    protected final Set<ServerLauncherContribution> contributions;

    protected AbstractServerLauncher(Set<ServerLauncherContribution> contributions) {
        this.contributions = contributions;
    }

    @Override
    public void initialize(String[] arguments) throws Exception {
        for (ServerLauncherContribution contribution : this.contributions) {
            contribution.onInitialize(arguments);
        }
    }

    @Override
    public void configure(String[] arguments) throws Exception {
        for (ServerLauncherContribution contribution : this.contributions) {
            contribution.onConfigure(arguments);
        }
    }
}
