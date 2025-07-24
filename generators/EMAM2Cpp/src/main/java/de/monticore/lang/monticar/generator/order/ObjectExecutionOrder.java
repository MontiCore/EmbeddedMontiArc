/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.order;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;

/**
 * Stores an object and its corresponding execution order.
 */
public class ObjectExecutionOrder<T> {
    protected T object;
    protected ExecutionOrder executionOrder;

    public ObjectExecutionOrder(T object, ExecutionOrder executionOrder) {
        this.object = object;
        this.executionOrder = executionOrder;
    }

    public T getObject() {
        return object;
    }

    public ExecutionOrder getExecutionOrder() {
        return executionOrder;
    }

    public void setObject(T object) {
        this.object = object;
    }

    public void setExecutionOrder(ExecutionOrder executionOrder) {
        this.executionOrder = executionOrder;
    }

}
