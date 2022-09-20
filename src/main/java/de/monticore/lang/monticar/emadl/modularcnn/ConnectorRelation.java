package de.monticore.lang.monticar.emadl.modularcnn;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTComponent;

public class ConnectorRelation {

    private ComponentInformation source;
    private ComponentInformation target;
    private String sourceValue;
    private String targetValue;

    public ConnectorRelation(ComponentInformation source, String sourceValue, ComponentInformation target, String targetValue){
        this.source = source;
        this.sourceValue = sourceValue;

        this.target = target;
        this.targetValue = targetValue;

    }

    public ComponentInformation getSource() {
        return source;
    }

    public ComponentInformation getTarget() {
        return target;
    }

    public String getSourceValue() {
        return sourceValue;
    }

    public String getTargetValue() {
        return targetValue;
    }
}
