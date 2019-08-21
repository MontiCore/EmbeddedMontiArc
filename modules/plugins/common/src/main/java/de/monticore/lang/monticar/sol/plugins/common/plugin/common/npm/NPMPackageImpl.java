/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm;

import com.google.inject.Inject;
import com.google.inject.assistedinject.Assisted;
import org.apache.commons.io.FileUtils;
import org.json.JSONObject;

import java.io.File;
import java.io.IOException;
import java.util.*;

public class NPMPackageImpl implements NPMPackage {
    protected final File path;
    protected final NPMPackageService resolver;
    protected final JSONObject content;
    protected final String name;

    protected List<NPMPackage> dependencies;
    protected SolPackage solPackage;
    protected TheiaPackage theiaPackage;

    @Inject
    protected NPMPackageImpl(@Assisted File path, NPMPackageService resolver) throws IOException {
        this.path = path;
        this.resolver = resolver;
        this.content = new JSONObject(FileUtils.readFileToString(path, "UTF-8"));
        this.name = this.hasAttribute("name") ? this.getAttribute("name") : null;
    }

    protected List<NPMPackage> fetchDependencies() {
        List<NPMPackage> packages = new ArrayList<>();

        if (this.hasAttribute("dependencies")) {
            Set<String> dependencies = this.<JSONObject>getAttribute("dependencies").keySet();

            dependencies.forEach(dependency -> this.resolver.resolve(dependency).ifPresent(packages::add));
        }

        return packages;
    }

    @Override
    public File getPath() {
        return this.path;
    }

    @Override
    public Optional<String> getName() {
        return Optional.ofNullable(this.name);
    }

    @Override
    public List<NPMPackage> getDependencies() {
        if (this.dependencies == null) this.dependencies = fetchDependencies();

        return Collections.unmodifiableList(this.dependencies);
    }

    @Override
    public boolean hasAttribute(String key) {
        return this.content.has(key);
    }

    @Override
    @SuppressWarnings("unchecked")
    public <T> T getAttribute(String key) {
        return (T)this.content.get(key);
    }

    @Override
    public boolean isTheiaPackage() {
        return this.hasAttribute("theiaExtensions");
    }

    @Override
    public boolean isSolPackage() {
        return this.hasAttribute("sol");
    }

    @Override
    public Optional<SolPackage> getAsSolPackage() {
        if (this.solPackage == null && this.isSolPackage()) this.solPackage = new SolPackageImpl(this);

        return Optional.ofNullable(this.solPackage);
    }

    @Override
    public Optional<TheiaPackage> getAsTheiaPackage() {
        if (this.theiaPackage == null && this.isTheiaPackage()) this.theiaPackage = new TheiaPackageImpl(this);

        return Optional.ofNullable(this.theiaPackage);
    }

    @Override
    public String toString() {
        return String.format("%s: %s", this.path, this.content);
    }
}
