/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.artifact._symboltable;

import de.monticore.lang.monticar.sol.grammars.common._ast.CommonLiterals;
import de.monticore.lang.monticar.sol.grammars.common._symboltable.SymbolWithProjectPath;
import de.monticore.lang.monticar.sol.grammars.environment._symboltable.DockerfileSymbolReference;
import de.monticore.lang.monticar.sol.grammars.environment._symboltable.SymbolWithEnvironment;
import de.monticore.lang.monticar.sol.grammars.language._symboltable.LanguageSymbolReference;
import de.monticore.lang.monticar.sol.grammars.language._symboltable.SymbolWithLanguages;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

public class ToolSymbol extends ToolSymbolTOP
        implements SymbolWithProjectPath, SymbolWithAlias, SymbolWithEnvironment, SymbolWithLanguages, SymbolWithArtifacts {
    protected final List<ArtifactSymbolReference> artifacts;
    protected final List<LanguageSymbolReference> languages;

    protected boolean virtual;
    protected String prefix;
    protected String suffix;
    protected String command;
    protected CommonLiterals origin;
    protected String path;
    protected String alias;
    protected DockerfileSymbolReference environment;

    public ToolSymbol(String name) {
        super(name);

        this.artifacts = new ArrayList<>();
        this.languages = new ArrayList<>();
    }

    public boolean isVirtual() {
        return this.virtual;
    }

    public void setVirtual(boolean virtual) {
        this.virtual = virtual;
    }

    public void setPrefix(String prefix) {
        this.prefix = prefix;
    }

    public Optional<String> getPrefix() {
        return Optional.ofNullable(this.prefix);
    }

    public void setSuffix(String suffix) {
        this.suffix = suffix;
    }

    public Optional<String> getSuffix() {
        return Optional.ofNullable(this.suffix);
    }

    public void setCommand(String command) {
        this.command = command;
    }

    public Optional<String> getCommand() {
        return Optional.ofNullable(this.command);
    }

    @Override
    public void addArtifact(ArtifactSymbolReference attribute) {
        this.artifacts.add(attribute);
    }

    @Override
    public List<ArtifactSymbolReference> getArtifacts() {
        return Collections.unmodifiableList(this.artifacts);
    }

    @Override
    public void setOrigin(CommonLiterals origin) {
        this.origin = origin;
    }

    @Override
    public Optional<CommonLiterals> getOrigin() {
        return Optional.ofNullable(this.origin);
    }

    @Override
    public void setPath(String path) {
        this.path = path;
    }

    @Override
    public Optional<String> getPath() {
        return Optional.ofNullable(this.path);
    }

    @Override
    public void setAlias(String alias) {
        this.alias = alias;
    }

    @Override
    public Optional<String> getAlias() {
        return Optional.ofNullable(this.alias);
    }

    @Override
    public void setEnvironment(DockerfileSymbolReference environment) {
        this.environment = environment;
    }

    @Override
    public Optional<DockerfileSymbolReference> getEnvironment() {
        return Optional.ofNullable(this.environment);
    }

    @Override
    public void addLanguage(LanguageSymbolReference language) {
        this.languages.add(language);
    }

    @Override
    public List<LanguageSymbolReference> getLanguages() {
        return Collections.unmodifiableList(this.languages);
    }
}
