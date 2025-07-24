/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.ide.plugin.generator.common.filter;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.mp.ModelPathService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm.NPMPackageService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm.SolPackage;
import de.monticore.symboltable.Symbol;

import java.util.Set;
import java.util.stream.Collectors;

@Singleton
public class LocalFilterImpl implements LocalFilter {
    protected final ModelPathService modelPath;
    protected final NPMPackageService packages;

    @Inject
    protected LocalFilterImpl(ModelPathService modelPath, NPMPackageService packages) {
        this.modelPath = modelPath;
        this.packages = packages;
    }

    @Override
    public <S extends Symbol> Set<S> filter(Set<S> configurations) {
        return configurations.stream()
                .filter(this::isSymbolLocal)
                .collect(Collectors.toSet());
    }

    protected boolean isSymbolLocal(Symbol symbol) {
        SolPackage solPackage = this.packages.getCurrentPackage()
                .orElseThrow(() -> new RuntimeException("Root Package could not be located."));

        return this.modelPath.isModelLocal(solPackage, symbol.getFullName(), "ide");
    }
}
