/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.common.state;

import com.google.common.base.Charsets;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.configuration.PluginConfiguration;
import org.apache.commons.io.FileUtils;
import org.json.JSONObject;

import java.io.File;
import java.io.IOException;
import java.util.Collection;
import java.util.Map;

public class State {
    protected final String identifier;
    protected final PluginConfiguration configuration;

    protected JSONObject values;

    public State(String identifier, PluginConfiguration configuration) throws IOException {
        this.identifier = identifier;
        this.configuration = configuration;

        this.load();
    }

    public void put(String key, boolean value) throws IOException {
        this.values.put(key, value);
        this.save();
    }

    public void put(String key, Collection value) throws IOException {
        this.values.put(key, value);
        this.save();
    }

    public void put(String key, double value) throws IOException {
        this.values.put(key, value);
        this.save();
    }

    public void put(String key, int value) throws IOException {
        this.values.put(key, value);
        this.save();
    }

    public void put(String key, long value) throws IOException {
        this.values.put(key, value);
        this.save();
    }

    public void put(String key, Map value) throws IOException {
        this.values.put(key, value);
        this.save();
    }

    public void put(String key, Object value) throws IOException {
        this.values.put(key, value);
        this.save();
    }

    public boolean getBoolean(String key) {
        return this.values.getBoolean(key);
    }

    public double getDouble(String key) {
        return this.values.getDouble(key);
    }

    public int getInt(String key) {
        return this.values.getInt(key);
    }

    public long getLong(String key) {
        return this.values.getLong(key);
    }

    protected File getFile() {
        return new File(this.configuration.getStatePath(), String.format("%s.json", this.identifier));
    }

    protected void load() throws IOException {
        File stateFile = this.getFile();

        if (stateFile.exists()) this.values = new JSONObject(FileUtils.readFileToString(stateFile, Charsets.UTF_8));
        else this.values = new JSONObject();
    }

    protected void save() throws IOException {
        FileUtils.writeStringToFile(this.getFile(), this.values.toString(2), Charsets.UTF_8);
    }
}
