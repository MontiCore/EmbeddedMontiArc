/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.roscpp.template;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;

public class RosCppCMakeListsModel {
    private String name;
    private String compName;
    private List<String> packages = new ArrayList<>();
    private List<String> excludeFindPackages = new ArrayList<>();

    public String getName() {
        return name;
    }

    public String getCompName() {
        return compName;
    }

    public List<String> getPackages() {
        return packages.stream()
                .sorted()
                .distinct()
                .collect(Collectors.toList());
    }

    public List<String> getExcludeFindPackages() {
        return excludeFindPackages;
    }

    public void addExcludeFindPackage(String excludeFindPackage) {
        this.excludeFindPackages.add(excludeFindPackage);
    }

    public RosCppCMakeListsModel(String name, String compName) {
        this.name = name;
        this.compName = compName;
    }

    public boolean addPackage(String s) {
        return packages.add(s);
    }

    public boolean addPackages(Collection<? extends String> collection) {
        return packages.addAll(collection);
    }
}
