package de.monticore.lang.monticar.generator.roscpp.template;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

public class RosCppCMakeListsModel {
    private String name;
    private String compName;
    private List<String> packages = new ArrayList<>();
    private List<String> additionalIncludes = new ArrayList<>();
    private List<String> additionalLibraries = new ArrayList<>();

    public String getName() {
        return name;
    }

    public String getCompName() {
        return compName;
    }

    public List<String> getPackages() {
        return packages;
    }

    public List<String> getAdditionalIncludes() {
        return additionalIncludes;
    }

    public List<String> getAdditionalLibraries() {
        return additionalLibraries;
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

    public boolean addInclude(String s) {
        return additionalIncludes.add(s);
    }

    public boolean addIncludes(Collection<? extends String> collection) {
        return additionalIncludes.addAll(collection);
    }

    public boolean addLibrary(String s) {
        return additionalLibraries.add(s);
    }

    public boolean addLibraries(Collection<? extends String> collection) {
        return additionalLibraries.addAll(collection);
    }
}
