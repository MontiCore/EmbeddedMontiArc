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

import java.util.Objects;
import java.util.Optional;

/**
 * This class is based on the NonVirtualBlock class from montiarc-executionOrder.
 * This class implements an NonVirtualBlock for the Execution Order
 * and is of the form: s:b{x,y}.
 * While s is the (system) index of the component/subsystem the block resides in. So s is 0 if the
 * block represents a top-level component.
 * In the sorted execution order b is the block position.
 * The indices of systems whose execution is controlled by this block are x,y .
 * The parameters s/b are necessary and the parameters of x/y are optional.
 */
public class NonVirtualBlock implements ExecutionOrder {

    private Integer s, b;
    private Optional<Integer> x = Optional.empty();
    private Optional<Integer> y = Optional.empty();

    private NonVirtualBlock nonVirtualBlock;

    @Override
    public String toString() {
        if (x.isPresent() && !y.isPresent()) {
            return String.format("%d:%d{%d}", s, b, x.get());
        } else if (!x.isPresent() && y.isPresent()) {
            return String.format("%d:%d{%d}", s, b, y.get());
        } else if (x.isPresent() && y.isPresent()) {
            return String.format("%d:%d{%d,%d}", s, b, x.get(), y.get());
        }
        return String.format("%d:%d", s, b);
    }

    public NonVirtualBlock(int s, int b) {
        this.s = s;
        this.b = b;
    }

    public void setX(Optional<Integer> x) {
        this.x = x;
    }

    public void setY(Optional<Integer> y) {
        this.y = y;
    }

    public int getS() {
        return this.s;
    }

    public int getB() {
        return this.b;
    }

    @Override
    public boolean equals(Object obj) {

        if (this == obj) {
            return true;
        }

        if ((obj == null) || (obj.getClass() != this.getClass())) {
            return false;
        }

        NonVirtualBlock that = (NonVirtualBlock) obj;
        return Objects.equals(this.s, that.s)
                && Objects.equals(this.b, that.b)
                && Objects.equals(this.x, that.x)
                && Objects.equals(this.y, that.y);
    }

    @Override
    public int hashCode() {
        int hc = 7;
        int hashMultiplier = 11;
        hc = hc * hashMultiplier + this.s;
        hc = hc * hashMultiplier + this.b;
        if (this.x.isPresent()) {
            hc = hc * hashMultiplier + this.x.get();
        }
        if (this.y.isPresent()) {
            hc = hc * hashMultiplier + this.y.get();
        }
        return hc;
    }

    /**
     * @param obj
     * @return -1 if this is smaller than obj, 1 if this is bigger than obj, 0 otherwise
     */
    public int compareTo(Object obj) {
        NonVirtualBlock that = (NonVirtualBlock) obj;
        int result = 0;
        if (this.s == that.s) {
            if (this.b == that.b) {
                result = compareSBEqual(that);
            } else if (this.b < that.b) {
                result = -1;
            } else if (this.b > that.b) {
                result = 1;
            }
        } else if (this.s < that.s) {
            result = -1;
        } else {
            result = 1;
        }
        return result;
    }

    public int compareSBEqual(NonVirtualBlock that) {
        int result = 0;
        if (this.x.isPresent() && that.x.isPresent() && this.x.get() == that.x.get()) {
            if (this.y.isPresent() && that.y.isPresent() && this.y.get() == that.y.get()) {
                result = 0;
            } else if (this.y.isPresent() || that.y.isPresent()) {
                result = compareYYEqual(that);
            }
        } else if (this.x.isPresent() || that.x.isPresent()) {
            if (this.x.isPresent() && that.x.isPresent()) {
                if (this.x.get() < that.x.get()) {
                    result = -1;
                } else if (this.x.get() > that.x.get()) {
                    result = 1;
                }
            } else if (this.x.isPresent() && !that.x.isPresent()) {
                result = 1;
            } else {
                result = -1;
            }
        }
        return result;
    }

    public int compareYYEqual(NonVirtualBlock that) {
        int result = 0;
        if (this.y.isPresent() && that.y.isPresent()) {
            if (this.y.get() < that.y.get()) {
                result = -1;
            } else if (this.y.get() > that.y.get()) {
                result = 1;
            }
        } else if (this.y.isPresent() && !that.y.isPresent()) {
            result = 1;
        } else {
            result = -1;
        }
        return result;
    }
}
