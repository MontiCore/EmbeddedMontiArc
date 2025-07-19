/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm;

import org.json.JSONArray;

import java.io.File;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;

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
    public Optional<String> getVersion() {
        return this.core.getVersion();
    }

    @Override
    public List<NPMPackage> getDependencies() {
        return this.core.getDependencies();
    }

    @Override
    public Set<NPMPackage> getAllDependencies() {
        return this.core.getAllDependencies();
    }

    @Override
    public <T> Optional<T> query(String query) {
        return this.core.query(query);
    }

    @Override
    public boolean isTheiaPackage() {
        return true;
    }

    @Override
    public boolean isSolPackage() {
        return this.core.isSolPackage();
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
        return this.<JSONArray>query("/theiaExtensions").orElse(new JSONArray());
    }

    @Override
    public Set<TheiaPackage> getTheiaDependencies() {
        return this.getDependencies().stream()
                .map(NPMPackage::getAsTheiaPackage)
                .filter(Optional::isPresent)
                .map(Optional::get)
                .collect(Collectors.toSet());
    }

    @Override
    public Set<TheiaPackage> getAllTheiaDependencies() {
        return this.getAllDependencies().stream()
                .map(NPMPackage::getAsTheiaPackage)
                .filter(Optional::isPresent)
                .map(Optional::get)
                .collect(Collectors.toSet());
    }

    @Override
    public String toString() {
        return this.core.toString();
    }
}
