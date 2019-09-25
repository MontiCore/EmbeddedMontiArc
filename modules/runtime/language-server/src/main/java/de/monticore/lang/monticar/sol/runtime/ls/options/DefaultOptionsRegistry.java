/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.runtime.ls.options;

import com.google.common.flogger.FluentLogger;
import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.runtime.ls.ls.ServerLauncherContribution;
import org.apache.commons.cli.Option;
import org.apache.commons.cli.Options;

import java.util.Set;

@Singleton
public class DefaultOptionsRegistry implements OptionsRegistry, ServerLauncherContribution {
    protected final FluentLogger logger;
    protected final Options options;
    protected final Set<OptionsContribution> contributions;

    @Inject
    protected DefaultOptionsRegistry(Set<OptionsContribution> contributions) {
        this.logger = FluentLogger.forEnclosingClass();
        this.options = new Options();
        this.contributions = contributions;
    }

    @Override
    public Options getOptions() {
        return this.options;
    }

    @Override
    public void registerOption(Option option) {
        this.options.addOption(option);
    }

    @Override
    public void onInitialize(String[] arguments) {
        this.logger.atInfo().log("Registering options...");

        for (OptionsContribution contribution : this.contributions) {
            contribution.registerOptions(this);
        }
    }
}
