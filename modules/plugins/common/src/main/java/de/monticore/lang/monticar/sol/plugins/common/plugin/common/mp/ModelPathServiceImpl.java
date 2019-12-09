/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.common.mp;

import com.google.inject.Singleton;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm.SolPackage;

import java.io.File;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

@Singleton
public class ModelPathServiceImpl implements ModelPathService {
    protected static final String MODELS = "models";

    @Override
    public ModelPath resolve(SolPackage root) {
        ModelPath modelPath = new ModelPath();
        List<SolPackage> allPackages = new ArrayList<>(root.getAllSolDependencies());

        allPackages.add(root);
        allPackages.stream().map(p -> p.getDirectoryAsPath(MODELS)).filter(Optional::isPresent)
                .map(Optional::get).forEach(modelPath::addEntry);

        return modelPath;
    }

    @Override
    public boolean isModelLocal(SolPackage root, String qualifiedName, String extension) {
        String normalizedExtension = extension.startsWith(".") ? extension : ("." + extension);
        String qualifiedPath = qualifiedName.replace(".", File.separator) + normalizedExtension;
        String modelsFolder = root.getDirectory(MODELS).orElse("sol/models");
        File modelFile = Paths.get(root.getPath().getParent(), modelsFolder, qualifiedPath).toFile();

        return modelFile.exists();
    }

    @Override
    public Optional<SolPackage> locateModel(SolPackage root, String qualifiedName, String extension) {
        List<SolPackage> allPackages = new ArrayList<>(root.getAllSolDependencies());

        allPackages.add(root);

        return allPackages.stream()
                .filter(solPackage -> this.isModelLocal(solPackage, qualifiedName, extension))
                .findFirst();
    }
}
