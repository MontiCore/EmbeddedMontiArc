/* (c) https://github.com/MontiCore/monticore */
package de.monticore.reporting.order;

import de.monticore.reporting.cocoReport.helper.CheckCoCoResult;
import de.monticore.reporting.helper.OrderableModelInfo;

public class ChildElement {
    private String referencedName;
    private OrderableModelInfo child;

    public ChildElement(String referencedName, OrderableModelInfo child){
        this.child = child;
        this.referencedName = referencedName;
    }

    public String getReferencedName() {
        return referencedName;
    }

    public OrderableModelInfo getChild() {
        return child;
    }
}
