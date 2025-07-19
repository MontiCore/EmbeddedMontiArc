/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.construct;

public class ComponentReplacement {
    private String parentComponent;
    private String oldInstanceName;
    private String packageName;
    private String type;
    private String newInstanceName;

    public ComponentReplacement(String parentComponent, String oldInstanceName, String packageName, String type, String newInstanceName) {
        this.parentComponent = parentComponent;
        this.oldInstanceName = oldInstanceName;
        this.packageName = packageName;
        this.newInstanceName = newInstanceName;
        this.type = type;
    }

    public String getParentComponent() {
        return parentComponent;
    }

    public String getOldInstanceName() {
        return oldInstanceName;
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

    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof ComponentReplacement)) return false;
        ComponentReplacement other = (ComponentReplacement) obj;
        if (!parentComponent.equals(other.getParentComponent())) return false;
        if (!oldInstanceName.equals(other.getOldInstanceName())) return false;
        if (!packageName.equals(other.getPackageName())) return false;
        if (!newInstanceName.equals(other.getNewInstanceName())) return false;
        if (!type.equals(other.getType())) return false;
        return true;
    }
}
