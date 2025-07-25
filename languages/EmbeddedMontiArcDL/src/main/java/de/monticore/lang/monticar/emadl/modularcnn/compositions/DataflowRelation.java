/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.emadl.modularcnn.compositions;

public class DataflowRelation {

    private ComponentInformation source;
    private ComponentInformation target;
    private String sourceValue;
    private String targetValue;

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

    public DataflowRelation(ComponentInformation source, String sourceValue, ComponentInformation target, String targetValue){
        this.source = source;
        this.sourceValue = sourceValue;

        this.target = target;
        this.targetValue = targetValue;

    }
}
