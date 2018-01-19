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

public class VectorSymbol extends CommonSymbol {

    private int start;
    private Optional<Integer> step;
    private int end;

    public static final VectorSymbol.VectorSymbolKind KIND = new VectorSymbol.VectorSymbolKind();

    public VectorSymbol(String name) {
        super(name, KIND);
    }

    public VectorSymbol(String name, int start, Optional<Integer> step, int end ){
        this(name);
        this.start=start;
        this.step = step;
        this.end= end;
    }

    public int getStart() {
        return start;
    }

    public void setStart(int start) {
        this.start = start;
    }

    public Optional<Integer> getStep() {
        return step;
    }

    public void setStep(Optional<Integer> step) {
        this.step = step;
    }

    public int getEnd() {
        return end;
    }

    public void setEnd(int end) {
        this.end = end;
    }

    public static class VectorSymbolKind implements SymbolKind {

        protected VectorSymbolKind() {
        }

        private static final String NAME = "Math-MatrixSymbolKind";
    }
}