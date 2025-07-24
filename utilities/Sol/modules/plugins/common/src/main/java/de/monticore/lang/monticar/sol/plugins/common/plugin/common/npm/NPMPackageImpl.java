/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm;

import com.google.inject.Inject;
import com.google.inject.assistedinject.Assisted;
import org.apache.commons.io.FileUtils;
import org.json.JSONObject;

import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.util.*;
import java.util.stream.Collectors;

public class NPMPackageImpl implements NPMPackage {
    protected final File path;
    protected final NPMPackageService resolver;
    protected final JSONObject content;
    protected final String name;
    protected final String version;

    protected List<NPMPackage> dependencies;
    protected SolPackage solPackage;
    protected TheiaPackage theiaPackage;

    @Inject
    protected NPMPackageImpl(@Assisted File path, NPMPackageService resolver) throws IOException {
        this.path = path;
        this.resolver = resolver;
        this.content = new JSONObject(FileUtils.readFileToString(path, StandardCharsets.UTF_8));
        this.name = this.<String>query("/name").orElse(null);
        this.version = this.<String>query("/version").orElse(null);
    }

    protected List<NPMPackage> fetchDependencies() {
        List<NPMPackage> packages = new ArrayList<>();
        Set<String> dependencies = this.<JSONObject>query("/dependencies")
                .map(JSONObject::keySet)
                .orElse(new HashSet<>());

        dependencies.forEach(dependency -> this.resolver.resolve(dependency).ifPresent(packages::add));
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
    public Optional<String> getVersion() {
        return Optional.ofNullable(this.version);
    }

    @Override
    public List<NPMPackage> getDependencies() {
        if (this.dependencies == null) this.dependencies = fetchDependencies();

        return Collections.unmodifiableList(this.dependencies);
    }

    @Override
    public Set<NPMPackage> getAllDependencies() {
       Set<NPMPackage> dependencies = this.getDependencies().stream()
               .flatMap(dependency -> dependency.getAllDependencies().stream())
               .collect(Collectors.toSet());

        dependencies.addAll(this.getDependencies());

        return dependencies;
    }

    @Override
    @SuppressWarnings("unchecked")
    public <T> Optional<T> query(String query) {
        return Optional.ofNullable((T)this.content.optQuery(query));
    }

    @Override
    public boolean isTheiaPackage() {
        return this.query("/theiaExtensions").isPresent();
    }

    @Override
    public boolean isSolPackage() {
        return this.query("/sol").isPresent();
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
