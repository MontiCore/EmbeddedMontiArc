/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.monticore.lang.math._symboltable;

import de.monticore.symboltable.CommonSymbol;
import de.monticore.symboltable.SymbolKind;

import java.util.Optional;

public class MatrixSymbol extends CommonSymbol {

    public static final MatrixSymbolKind KIND = new MatrixSymbolKind();

    // number like `2` is dimension 1x1
    int row = 1;
    int col = 1;
    Optional<Double> min = Optional.empty();
    Optional<Double> max = Optional.empty();
    Optional<Double> step = Optional.empty();
    boolean isComplex = false;

    public MatrixSymbol(String name) {
        super(name, KIND);
    }

    public MatrixSymbol(String name, int row, int col, Optional<Double> min, Optional<Double> max, Optional<Double> step, boolean isComplex) {
        this(name);
        this.row = row;
        this.col = col;
        this.min = min;
        this.max = max;
        this.step = step;
        this.isComplex = isComplex;
    }

    public int getRow() {
        return row;
    }

    public void setRow(int row) {
        this.row = row;
    }

    public int getCol() {
        return col;
    }

    public void setCol(int col) {
        this.col = col;
    }

    public Optional<Double> getMin() {
        return min;
    }

    public void setMin(Optional<Double> min) {
        this.min = min;
    }

    public Optional<Double> getMax() {
        return max;
    }

    public void setMax(Optional<Double> max) {
        this.max = max;
    }

    public Optional<Double> getStep() {
        return step;
    }

    public void setStep(Optional<Double> step) {
        this.step = step;
    }

    public boolean isComplex() {
        return isComplex;
    }

    public void setComplex(boolean complex) {
        isComplex = complex;
    }


    public static class MatrixSymbolKind implements SymbolKind {

        protected MatrixSymbolKind() {
        }

        private static final String NAME = "Math-MatrixSymbolKind";
    }
}
