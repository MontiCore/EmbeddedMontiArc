/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm;

import org.json.JSONException;
import org.json.JSONObject;

import java.io.File;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
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
        return this.core.hasAttribute("theiaExtensions");
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
        try {
            return Optional.of(this.<JSONObject>getAttribute("sol").getJSONObject("directories").getString(identifier));
        } catch(JSONException exception) {
            return Optional.empty();
        }
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
        List<SolPackage> directDependencies = this.getSolDependencies();
        List<SolPackage> transitiveDependencies = new ArrayList<>(directDependencies);

        directDependencies.forEach(dependency -> transitiveDependencies.addAll(dependency.getAllSolDependencies()));

        return Collections.unmodifiableList(transitiveDependencies);
    }

    @Override
    public String toString() {
        return this.core.toString();
    }
}
