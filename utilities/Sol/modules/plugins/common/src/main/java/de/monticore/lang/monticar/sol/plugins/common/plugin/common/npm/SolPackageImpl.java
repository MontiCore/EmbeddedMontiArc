/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm;

import org.json.JSONArray;

import java.io.File;
import java.nio.file.Path;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;

public class SolPackageImpl implements SolPackage {
    protected final NPMPackage core;

    protected SolPackageImpl(NPMPackage core) {
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
        return this.core.isTheiaPackage();
    }

    @Override
    public boolean isSolPackage() {
        return true;
    }

    @Override
    public Optional<SolPackage> getAsSolPackage() {
        return Optional.of(this);
    }

    @Override
    public Optional<TheiaPackage> getAsTheiaPackage() {
        return this.core.getAsTheiaPackage();
    }

    @Override
    public Optional<String> getDirectory(String identifier) {
        return this.query(String.format("/sol/directories/%s", identifier));
    }

    @Override
    public Optional<Path> getDirectoryAsPath(String identifier) {
        return this.getDirectory(identifier).map(relativePath ->
                this.getPath().toPath().getParent().resolve(relativePath));
    }

    @Override
    public List<SolPackage> getSolDependencies() {
        return this.getDependencies().stream()
                .filter(NPMPackage::isSolPackage)
                .map(NPMPackage::getAsSolPackage)
                .map(Optional::get)
                .collect(Collectors.toList());
    }

    @Override
    public List<SolPackage> getAllSolDependencies() {
        return this.getAllDependencies().stream()
                .filter(NPMPackage::isSolPackage)
                .map(NPMPackage::getAsSolPackage)
                .map(Optional::get)
                .collect(Collectors.toList());
    }

    @Override
    public Optional<JSONArray> getExtensions() {
        return this.query("/sol/extensions");
    }

    @Override
    public String toString() {
        return this.core.toString();
    }
}
