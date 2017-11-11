package de.monticore.lang.monticar.generator.cpp.resolver;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;

public class ResolverFactory {

    private Path[] modelPaths;

    public ResolverFactory(Path... modelPaths) {
        this.modelPaths = modelPaths;
    }

    public Resolver get() {
        SymTabCreator symTabCreator = new SymTabCreator(modelPaths);
        return new Resolver(symTabCreator.createSymTab());
    }

    public void addDefaultTypes(Path path) {
        modelPaths = Arrays.copyOf(modelPaths, modelPaths.length + 1);
        modelPaths[modelPaths.length - 1] = path;
    }


}
