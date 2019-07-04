/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.common.state;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.configuration.PluginConfiguration;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

@Singleton
public class StateFactoryImpl implements StateFactory {
    protected final PluginConfiguration configuration;
    protected final Map<String, State> states;

    @Inject
    public StateFactoryImpl(PluginConfiguration configuration) {
        this.configuration = configuration;
        this.states = new HashMap<>();
    }

    @Override
    public State create(String identifier) throws IOException {
        return this.states.containsKey(identifier) ?
                this.states.get(identifier) :
                this.states.put(identifier, new State(identifier, this.configuration));
    }
}
