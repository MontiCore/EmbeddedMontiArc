/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.construct;

public class ConnectorReplacement {
    private String parentComponent;
    private String sourceComponent;
    private String sourcePort;
    private String targetComponent;
    private String targetPort;
    private String source;
    private String target;

    public ConnectorReplacement(String parentComponent, String sourceComponent, String sourcePort, String targetComponent, String targetPort) {
        this.parentComponent = parentComponent;
        this.sourceComponent = sourceComponent;
        this.sourcePort = sourcePort;
        this.targetComponent = targetComponent;
        this.targetPort = targetPort;
        this.source = (sourceComponent != null && !sourceComponent.equals("")) ? sourceComponent + "." + sourcePort : sourcePort;
        this.target = (targetComponent != null && !targetComponent.equals("")) ? targetComponent + "." + targetPort : targetPort;
    }

    public ConnectorReplacement(String parentComponent, String source, String target) {
        this.parentComponent = parentComponent;
        this.source = source;
        this.target = target;
        this.sourceComponent = source.contains(".") ? source.substring(0, source.indexOf(".")) : "";
        this.sourcePort = source.contains(".") ? source.substring(source.indexOf(".") + 1) : source;
        this.targetComponent = target.contains(".") ? target.substring(0, target.indexOf(".")) : "";
        this.targetPort = target.contains(".") ? target.substring(target.indexOf(".") + 1) : target;
    }

    public String getParentComponent() {
        return parentComponent;
    }

    public String getSourceComponent() {
        return sourceComponent;
    }

    public String getSourcePort() {
        return sourcePort;
    }

    public String getTargetComponent() {
        return targetComponent;
    }

    public String getTargetPort() {
        return targetPort;
    }

    public String getSource() {
        return source;
    }

    public String getTarget() {
        return target;
    }

    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof ConnectorReplacement)) return false;
        ConnectorReplacement other = (ConnectorReplacement) obj;
        if (!parentComponent.equals(other.getParentComponent())) return false;
        if (!sourceComponent.equals(other.getSourceComponent())) return false;
        if (!sourcePort.equals(other.getSourcePort())) return false;
        if (!source.equals(other.getSource())) return false;
        if (!targetComponent.equals(other.getTargetComponent())) return false;
        if (!targetPort.equals(other.getTargetPort())) return false;
        if (!target.equals(other.getTarget())) return false;
        return true;
    }
}
