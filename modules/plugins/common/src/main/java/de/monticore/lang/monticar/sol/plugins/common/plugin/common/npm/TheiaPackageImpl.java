/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm;

import org.json.JSONArray;

import java.io.File;
import java.util.List;
import java.util.Optional;

public class TheiaPackageImpl implements TheiaPackage {
    protected final NPMPackage core;

    protected TheiaPackageImpl(NPMPackage core) {
        this.core = core;
    }

    @Override
    public File getPath() {
        return this.core.getPath();
    }

    @Override
    public Optional<String> getName() {
        return this.core.getName();
    }

    @Override
    public List<NPMPackage> getDependencies() {
        return this.core.getDependencies();
    }

    @Override
    public boolean hasAttribute(String key) {
        return this.core.hasAttribute(key);
    }

    @Override
    public <T> T getAttribute(String key) {
        return this.core.getAttribute(key);
    }

    @Override
    public boolean isTheiaPackage() {
        return true;
    }

    @Override
    public boolean isSolPackage() {
        return this.core.hasAttribute("sol");
    }

    @Override
    public Optional<SolPackage> getAsSolPackage() {
        return this.core.getAsSolPackage();
    }

    @Override
    public Optional<TheiaPackage> getAsTheiaPackage() {
        return Optional.of(this);
    }

    @Override
    public JSONArray getExtensions() {
        return this.getAttribute("theiaExtensions");
    }

    @Override
    public String toString() {
        return this.core.toString();
    }
}
