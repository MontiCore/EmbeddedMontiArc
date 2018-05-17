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
package de.monticore.lang.math.math._symboltable;

//import de.monticore.lang.math.math._ast.ASTMathStatement;
import de.monticore.lang.monticar.ranges._ast.ASTRange;
import de.monticore.symboltable.CommonSymbol;

import java.util.List;
import java.util.Optional;

/**
 * @author math-group
 */
public class MathVariableDeclarationSymbol extends CommonSymbol {

   // private Optional<ASTMathStatement> wholeLine = Optional.empty();
    /**
     * matrix properties
     */
    private List<String> matrixProperties;

    /**
     * a value
     */
    private MathValue value;

    public static MathVariableDeclarationKind KIND = new MathVariableDeclarationKind();

    /**
     * a range
     */
    private ASTRange range;

    /**
     * booleans which represents the type of the value
     */
    private boolean isComplex = false;
    private boolean isRational = false;
    private boolean isBoolean = false;

    /**
     * dimensions for instance for matrices, otherwise 1x1
     */
    private List<Integer> dimensions;

    /**
     * counter to create names for anonymus symbols
     */
    public static long anonymusNameCounter = 1;

    /**
     * create a new symbol with the name and values given by its arguments
     */
    public MathVariableDeclarationSymbol(String name, ASTRange range, List<Integer> dimensions, List<String> matrixProperties) {
        super(name, KIND);
        this.range = range;
        this.dimensions = dimensions;
        this.matrixProperties = matrixProperties;
        this.value = null;
        anonymusNameCounter++;
    }

    public MathVariableDeclarationSymbol(String name, ASTRange range, List<Integer> dimensions, List<String> matrixProperties, MathValue value, boolean isComplex, boolean isRational) {
        super(name, KIND);
        this.range = range;
        this.dimensions = dimensions;
        this.matrixProperties = matrixProperties;
        this.isComplex = isComplex;
        this.isRational = isRational;
        this.value = value;
        anonymusNameCounter++;
    }

    public MathVariableDeclarationSymbol(String name, ASTRange range, List<Integer> dimensions, List<String> matrixProperties, MathValue value, boolean isBoolean) {
        super(name, KIND);
        this.range = range;
        this.dimensions = dimensions;
        this.matrixProperties = matrixProperties;
        this.isBoolean = isBoolean;
        this.value = value;
        anonymusNameCounter++;
    }

    public MathVariableDeclarationSymbol(String name, List<Integer> dimensions, List<String> matrixProperties, MathValue value, boolean isComplex, boolean isRational) {
        super(name, KIND);
        this.dimensions = dimensions;
        this.matrixProperties = matrixProperties;
        this.isComplex = isComplex;
        this.isRational = isRational;
        this.value = value;
        this.range = null;
        anonymusNameCounter++;
    }

    public MathVariableDeclarationSymbol(String name, List<Integer> dimensions, MathValue value, boolean isComplex, boolean isRational) {
        super(name, KIND);
        this.dimensions = dimensions;
        this.isComplex = isComplex;
        this.isRational = isRational;
        this.value = value;
        this.matrixProperties = null;
        this.range = null;
        anonymusNameCounter++;
    }

    public MathVariableDeclarationSymbol(String name, List<Integer> dimensions, MathValue value) {
        super(name, KIND);
        this.dimensions = dimensions;
        this.value = value;
        this.matrixProperties = null;
        this.range = null;
        anonymusNameCounter++;
    }

    public MathVariableDeclarationSymbol(String name, MathValue value, boolean isComplex, boolean isRational) {
        super(name, KIND);
        this.isComplex = isComplex;
        this.isRational = isRational;
        this.value = value;
        this.range = null;
        this.matrixProperties = null;
        this.dimensions = null;
        anonymusNameCounter++;
    }

    public MathVariableDeclarationSymbol(String name, MathValue value, boolean isBoolean) {
        super(name, KIND);
        this.value = value;
        this.isBoolean = isBoolean;
        this.range = null;
        this.dimensions = null;
        this.matrixProperties = null;
        anonymusNameCounter++;
    }

    public MathVariableDeclarationSymbol(String name, MathValue value, List<String> matrixProperties) {
        super(name, KIND);
        this.value = value;
        this.range = null;
        this.dimensions = null;
        this.matrixProperties = matrixProperties;
        anonymusNameCounter++;
    }

    public MathVariableDeclarationSymbol(String name, MathValue value) {
        super(name, KIND);
        this.value = value;
        this.range = null;
        this.dimensions = null;
        this.matrixProperties = null;
        anonymusNameCounter++;
    }
/*
    public MathVariableDeclarationSymbol(String name, ASTMathStatement wholeLine) {
        super(name, KIND);
        this.value = value;
        this.range = null;
        this.dimensions = null;
        this.matrixProperties = null;
        anonymusNameCounter++;
        setWholeLine(wholeLine);
    }


    public Optional<ASTMathStatement> getWholeLine() {
        return wholeLine;
    }

    public void setWholeLine(ASTMathStatement mathStatement) {
        this.wholeLine = Optional.ofNullable(mathStatement);
    }


    /**
     * get the dimensions as an integer list
     *
     * @return dimensions
     */
    public List<Integer> getDimensions() {
        return this.dimensions;
    }

    /**
     * checks if the type of this value is complex (complex number)
     *
     * @return TRUE, if its a complex value, otherwise FALSE
     */
    public boolean isComplex() {
        return this.isComplex;
    }

    /**
     * checks if the type of this value is boolean (boolean value)
     *
     * @return TRUE, if its a boolean value, otherwise FALSE
     */
    public boolean isBoolean() {
        return this.isBoolean;
    }

    /**
     * checks if the type of this value is rational (rational value)
     *
     * @return TRUE, if its a rational value, otherwise FALSE
     */
    public boolean isRational() {
        return this.isRational;
    }

    /**
     * get the value of this symbol
     *
     * @return value
     */
    public MathValue getValue() {
        return this.value;
    }

    /**
     * set the value of this symbol given by its argument
     */
    public void setValue(MathValue value) {
        this.value = value;
    }

    /**
     * @return matrix property such as diag or inv
     */
    public List<String> getMatrixProperties() {
        return matrixProperties;
    }

    /**
     * get the range type for this symbol
     *
     * @return range of his type
     */
    public ASTRange getRange() {
        return range;
    }

}
