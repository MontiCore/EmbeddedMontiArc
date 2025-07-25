/* (c) https://github.com/MontiCore/monticore */
package de.monticore.reporting.helper;

import de.monticore.reporting.order.ChildElement;

import java.io.File;
import java.util.LinkedList;
import java.util.List;

public abstract class OrderableModelInfo extends CommonModelInfo {

    public static String erroredString = "" + ((char) 160) + "Errored";

    private String rootName = "";
    private File rootFile = null;
    private String project = "";
    private List<ChildElement> children = new LinkedList<>();
    private List<OrderableModelInfo> parents = new LinkedList<>();
    private boolean isAtomic = true;

    private boolean errorResult = false;
    private boolean mainPackage = false;

    public OrderableModelInfo(File modelFile) {
        super(modelFile);
    }

    public OrderableModelInfo(String modelFile) {
        super(new File(modelFile));
    }

    public String getProject() {
        return project;
    }

    public void setProject(String project) {
        this.project = project;
    }

    public List<ChildElement> getChildren() {
        return children;
    }

    public void addChild(ChildElement childElement) {
        this.children.add(childElement);
    }

    public boolean isErrorResult() {
        return errorResult;
    }

    public void setErrorResult(boolean errorResult) {
        this.errorResult = errorResult;
    }

    public boolean isMainPackage() {
        return mainPackage;
    }

    public void setMainPackage(boolean mainPackage) {
        this.mainPackage = mainPackage;
    }

    public List<OrderableModelInfo> getParents(){
        return parents;
    }

    public void addParent(OrderableModelInfo parent) {
        this.parents.add(parent);
    }

    public abstract void setChildInfo();

    public String getRootName() {
        return rootName;
    }

    public void setRootName(String rootName) {
        this.rootName = rootName;
    }

    public File getRootFile() {
        return rootFile;
    }

    public void setRootFile(File rootFile) {
        this.rootFile = rootFile;
        this.rootName = rootFile.getName();
    }

    public boolean isAtomic() {
        return isAtomic;
    }

    public void setAtomic(boolean atomic) {
        isAtomic = atomic;
    }
}
