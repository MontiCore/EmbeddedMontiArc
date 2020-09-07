/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.construct;

import java.util.HashSet;
import java.util.Set;

public class Replacement {
    Set<ComponentReplacement> componentReplacements;
    Set<ConnectorReplacement> connectorReplacements;
    Set<PortReplacement> portReplacements;

    public Replacement() {
        this.componentReplacements = new HashSet<>();
        this.connectorReplacements = new HashSet<>();
        this.portReplacements = new HashSet<>();
    }

    public Set<ComponentReplacement> getComponentReplacements() {
        return componentReplacements;
    }

    public Set<ConnectorReplacement> getConnectorReplacements() {
        return connectorReplacements;
    }

    public Set<PortReplacement> getPortReplacements() {
        return portReplacements;
    }

    public void add(ComponentReplacement componentReplacement) {
        this.componentReplacements.add(componentReplacement);
    }

    public void add(ConnectorReplacement connectorReplacement) {
        this.connectorReplacements.add(connectorReplacement);
    }

    public void add(PortReplacement portReplacement) {
        this.portReplacements.add(portReplacement);
    }

    public void remove(ComponentReplacement componentReplacement) {
        this.componentReplacements.remove(componentReplacement);
    }

    public void remove(ConnectorReplacement connectorReplacement) {
        this.connectorReplacements.remove(connectorReplacement);
    }

    public void remove(PortReplacement portReplacement) {
        this.portReplacements.remove(portReplacement);
    }
}
