/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.artifact._symboltable;

import de.monticore.lang.monticar.sol.grammars.common._ast.CommonLiterals;
import de.monticore.lang.monticar.sol.grammars.common._symboltable.SymbolWithProjectPath;

import java.util.Optional;

public class ArtifactSymbol extends ArtifactSymbolTOP implements SymbolWithAlias, SymbolWithProjectPath {
    protected String alias;
    protected CommonLiterals origin;
    protected String path;

    public ArtifactSymbol(String name) {
        super(name);
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
}
