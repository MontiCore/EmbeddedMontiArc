package de.monticore.reporting.order;

import de.monticore.reporting.cocoReport.helper.CheckCoCoResult;

public class ChildElement {
    private String referencedName;
    private CheckCoCoResult child;

    public ChildElement(String referencedName, CheckCoCoResult child){
        this.child = child;
        this.referencedName = referencedName;
    }

    public String getReferencedName() {
        return referencedName;
    }

    public CheckCoCoResult getChild() {
        return child;
    }
}
