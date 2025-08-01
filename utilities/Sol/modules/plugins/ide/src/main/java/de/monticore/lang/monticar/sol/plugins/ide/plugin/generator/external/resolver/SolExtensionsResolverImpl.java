/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.ide.plugin.generator.external.resolver;

import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm.SolPackage;
import org.json.JSONArray;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

@Singleton
public class SolExtensionsResolverImpl implements SolExtensionsResolver {
    @Override
    public List<String> resolveExtensions(SolPackage rootPackage, String identifier) {
        return this.resolveAllPackagesWithExtensions(rootPackage).stream()
                .flatMap(p -> this.getExtensions(p, identifier).stream())
                .collect(Collectors.toList());
    }

    @Override
    public Set<SolPackage> resolveAllPackagesWithExtensions(SolPackage rootPackage) {
        List<SolPackage> allPackages = new ArrayList<>(rootPackage.getAllSolDependencies());

        allPackages.add(rootPackage);

        return allPackages.stream()
                .filter(p -> p.getExtensions().isPresent())
                .collect(Collectors.toSet());
    }

    protected List<String> getExtensions(SolPackage solPackage, String identifier) {
        List<String> result = new ArrayList<>();
        String packageName = solPackage.getName().orElseThrow(() -> new RuntimeException("Package does not have a name"));
        JSONArray extensions = solPackage.getExtensions().orElse(new JSONArray());

        for (int i = 0; i < extensions.length(); i++) {
            JSONObject extension = extensions.getJSONObject(i);

            if (extension.has(identifier)) result.add(String.format("%s/%s", packageName, extension.get(identifier)));
        }

        return result;
    }
}
