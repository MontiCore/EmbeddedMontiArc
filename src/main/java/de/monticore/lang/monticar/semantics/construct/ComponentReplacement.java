/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.construct;

public class ComponentReplacement {
    private String parentComponent;
    private String instanceNameToReplace;
    private String packageName;
    private String type;
    private String newInstanceName;

    public ComponentReplacement(String parentComponent, String instanceNameToReplace, String packageName, String type, String newInstanceName) {
        this.parentComponent = parentComponent;
        this.instanceNameToReplace = instanceNameToReplace;
        this.packageName = packageName;
        this.newInstanceName = newInstanceName;
        this.type = type;
    }

    public String getParentComponent() {
        return parentComponent;
    }

    public String getInstanceNameToReplace() {
        return instanceNameToReplace;
    }

    public String getPackageName() {
        return packageName;
    }

    public String getNewInstanceName() {
        return newInstanceName;
    }

    public String getType() {
        return type;
    }
}
