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
}
