/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.construct;

import java.util.Set;

public class Replacement {
    Set<ComponentReplacement> componentReplacements;
    Set<ConnectorReplacement> connectorReplacements;
    Set<PortReplacement> portReplacements;

    public Replacement(Set<ComponentReplacement> componentReplacements, Set<ConnectorReplacement> connectorReplacements, Set<PortReplacement> portReplacements) {
        this.componentReplacements = componentReplacements;
        this.connectorReplacements = connectorReplacements;
        this.portReplacements = portReplacements;
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
}
