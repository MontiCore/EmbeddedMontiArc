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
    private int row = 1;
    private int col = 1;
    private Optional<Double> minDimension = Optional.empty();
    private Optional<Double> maxDimension = Optional.empty();
    private Optional<Double> stepDimension = Optional.empty();
    private boolean isComplex = false;

    public MatrixSymbol(String name) {
        super(name, KIND);
    }

    public MatrixSymbol(String name, int row, int col, Optional<Double> minDimension, Optional<Double> maxDimension, Optional<Double> stepDimension, boolean isComplex) {
        this(name);
        this.row = row;
        this.col = col;
        this.minDimension = minDimension;
        this.maxDimension = maxDimension;
        this.stepDimension = stepDimension;
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
        return minDimension;
    }

    public void setMin(Optional<Double> min) {
        this.minDimension = minDimension;
    }

    public Optional<Double> getMax() {
        return maxDimension;
    }

    public void setMax(Optional<Double> max) {
        this.maxDimension = maxDimension;
    }

    public Optional<Double> getStep() {
        return stepDimension;
    }

    public void setStep(Optional<Double> step) {
        this.stepDimension = stepDimension;
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
