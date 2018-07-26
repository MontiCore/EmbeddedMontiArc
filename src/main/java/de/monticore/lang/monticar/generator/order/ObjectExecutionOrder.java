/*
 * ******************************************************************************
 * MontiCore Language Workbench, www.monticore.de
 * Copyright (c) 2017, MontiCore, All rights reserved.
 *
 * This project is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3.0 of the License, or (at your option) any later version.
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * ******************************************************************************
 */

package de.monticore.lang.monticar.generator.order;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;

/**
 * Stores an object and its corresponding execution order.
 * @author Sascha
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
