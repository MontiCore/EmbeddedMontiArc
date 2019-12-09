/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.ide.plugin.generator.external.resolver;

import de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm.SolPackage;

import java.util.List;
import java.util.Set;

public interface SolExtensionsResolver {
    List<String> resolveExtensions(SolPackage rootPackage, String identifier);
    Set<SolPackage> resolveAllPackagesWithExtensions(SolPackage rootPackage);

    default List<String> resolveMainExtensions(SolPackage rootPackage) {
        return this.resolveExtensions(rootPackage, "main");
    }

    default List<String> resolveRendererExtensions(SolPackage rootPackage) {
        return this.resolveExtensions(rootPackage, "renderer");
    }
}
