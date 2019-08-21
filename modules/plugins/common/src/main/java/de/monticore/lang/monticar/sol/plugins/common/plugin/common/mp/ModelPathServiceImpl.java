/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.common.mp;

import com.google.inject.Singleton;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm.SolPackage;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

@Singleton
public class ModelPathServiceImpl implements ModelPathService {
    @Override
    public ModelPath resolve(SolPackage root) {
        ModelPath modelPath = new ModelPath();
        List<SolPackage> allPackages = new ArrayList<>(root.getAllSolDependencies());

        allPackages.add(root);
        allPackages.stream().map(p -> p.getDirectoryAsPath("models")).filter(Optional::isPresent)
                .map(Optional::get).forEach(modelPath::addEntry);

        return modelPath;
    }
}
