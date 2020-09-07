/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.construct;

public class PortReplacement {
    private String component;
    private String type;
    private String name;
    private boolean incoming;

    public PortReplacement(String component, String type, String name, boolean incoming) {
        this.component = component;
        this.type = type;
        this.name = name;
        this.incoming = incoming;
    }

    public String getComponent() {
        return component;
    }

    public String getType() {
        return type;
    }

    public String getName() {
        return name;
    }

    public boolean isIncoming() {
        return incoming;
    }

    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof PortReplacement)) return false;
        PortReplacement other = (PortReplacement) obj;
        if (!component.equals(other.getComponent())) return false;
        if (!type.equals(other.getType())) return false;
        if (!name.equals(other.getName())) return false;
        if (incoming != other.isIncoming()) return false;
        return true;
    }
}
