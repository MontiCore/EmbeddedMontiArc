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
package de.monticore.lang.math;

import de.monticore.lang.math._matrixprops.MatrixProperties;
import de.monticore.lang.math._symboltable.expression.MathValueSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixArithmeticValueSymbol;
import de.monticore.symboltable.Scope;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

import java.util.ArrayList;

import static org.junit.Assert.*;

/**
 * Created by Philipp Goerick on 26.07.2017.
 */

public class MatrixPropertiesTest extends MathSymbolTableCreatorTest {

    private Scope symTab;
    @Before
    public void setUp() {
        symTab = createSymTab("src/test/resources");
    }

    /**
     * checks if m1 is positive definite and invertible
     */
    @Test
    public void posDefTest() {
        //Q^{3,3} m1 = [2 -1 0;-1 2 -1;0 -1 2];
        final MathValueSymbol m1 = symTab.<MathValueSymbol>resolve
                ("symtab.SymtabMatrixProperties.m1", MathValueSymbol.KIND).orElse(null);
        assertNotNull(m1);
        MathMatrixArithmeticValueSymbol symbol = ((MathMatrixArithmeticValueSymbol)m1.getValue());
        ArrayList<MatrixProperties> properties = symbol.getMatrixProperties();
        assertTrue(symbol.getMatrixProperties().contains(MatrixProperties.Square));
        assertTrue(properties.contains(MatrixProperties.Norm));
        assertTrue(properties.contains(MatrixProperties.Herm));
        assertTrue(properties.contains(MatrixProperties.PosDef));
        assertTrue(properties.contains(MatrixProperties.Invertible));
        assertFalse(properties.contains(MatrixProperties.NegDef));
        assertFalse(properties.contains(MatrixProperties.Diag));
    }

    /**
     * checks if m2 is negative definite and invertible
     */
    @Test
    public void negDefTest() {
        //Q^{3,3} m2 = [-5 0 0;0 -4 1;0 1 -4];
        final MathValueSymbol m2 = symTab.<MathValueSymbol>resolve
                ("symtab.SymtabMatrixProperties.m2", MathValueSymbol.KIND).orElse(null);
        assertNotNull(m2);
        MathMatrixArithmeticValueSymbol symbol = ((MathMatrixArithmeticValueSymbol)m2.getValue());
        ArrayList<MatrixProperties> properties = symbol.getMatrixProperties();
        assertTrue(properties.contains(MatrixProperties.Square));
        assertTrue(properties.contains(MatrixProperties.Norm));
        assertTrue(properties.contains(MatrixProperties.Herm));
        assertTrue(properties.contains(MatrixProperties.NegDef));
        assertTrue(properties.contains(MatrixProperties.Invertible));
        assertFalse(properties.contains(MatrixProperties.PosDef));
        assertFalse(properties.contains(MatrixProperties.Diag));
    }

    /**
     * checks if m3 is diagonal and invertible
     */
    @Test
    public void diagTest(){
        //Q^{3,3} m3 = [2 0 0;0 2 0;0 0 2];
        final MathValueSymbol m3 = symTab.<MathValueSymbol>resolve
                ("symtab.SymtabMatrixProperties.m3", MathValueSymbol.KIND).orElse(null);
        assertNotNull(m3);
        MathMatrixArithmeticValueSymbol symbol = ((MathMatrixArithmeticValueSymbol)m3.getValue());
        ArrayList<MatrixProperties> properties = symbol.getMatrixProperties();
        assertTrue(properties.contains(MatrixProperties.Square));
        assertTrue(properties.contains(MatrixProperties.Norm));
        assertTrue(properties.contains(MatrixProperties.Diag));
        assertTrue(properties.contains(MatrixProperties.Herm));
        assertTrue(properties.contains(MatrixProperties.Invertible));
    }

    /**
     * checks if m4 is diagonal and not invertible
     */
    @Test
    public void notInvertibleTest(){
        //Q^{3,3} m4 = [2 0 0;0 0 0;0 0 2];
        final MathValueSymbol m4 = symTab.<MathValueSymbol>resolve
                ("symtab.SymtabMatrixProperties.m4", MathValueSymbol.KIND).orElse(null);
        assertNotNull(m4);
        MathMatrixArithmeticValueSymbol symbol = ((MathMatrixArithmeticValueSymbol)m4.getValue());
        ArrayList<MatrixProperties> properties = symbol.getMatrixProperties();
        assertTrue(properties.contains(MatrixProperties.Square));
        assertTrue(properties.contains(MatrixProperties.Norm));
        assertTrue(properties.contains(MatrixProperties.Diag));
        assertTrue(properties.contains(MatrixProperties.Herm));
        assertFalse(properties.contains(MatrixProperties.Invertible));
    }

    /**
     * checks if m5 is skew-hermitian and invertible (not possible until complex matrix support)
     */
    @Ignore @Test
    public void skewHermitianTest() {
        //C^{3,3} m5 = [i -1+2i 1;1+2i 0 -2-i;-1 -2-i -2i];
        final MathValueSymbol m5 = symTab.<MathValueSymbol>resolve
                ("symtab.SymtabMatrixProperties.m5", MathValueSymbol.KIND).orElse(null);
        assertNotNull(m5);
        MathMatrixArithmeticValueSymbol symbol = ((MathMatrixArithmeticValueSymbol)m5.getValue());
        ArrayList<MatrixProperties> properties = symbol.getMatrixProperties();
        assertTrue(properties.contains(MatrixProperties.Square));
        assertTrue(properties.contains(MatrixProperties.Norm));
        assertTrue(properties.contains(MatrixProperties.Diag));
        assertTrue(properties.contains(MatrixProperties.SkewHerm));
        assertTrue(properties.contains(MatrixProperties.Invertible));
    }

    /**
     * checks if m6 is hermitian (not possible until complex matrix support)
     */
    @Ignore @Test
    public void hermitianTest(){
        //C^{3,3} m6 = [2 1-i 4+2i;1+i 1 2-i;4-2i 2+i -2];
        final MathValueSymbol m6 = symTab.<MathValueSymbol>resolve
                ("symtab.SymtabMatrixProperties.m6", MathValueSymbol.KIND).orElse(null);
        assertNotNull(m6);
        MathMatrixArithmeticValueSymbol symbol = ((MathMatrixArithmeticValueSymbol)m6.getValue());
        ArrayList<MatrixProperties> properties = symbol.getMatrixProperties();
        assertTrue(properties.contains(MatrixProperties.Square));
        assertTrue(properties.contains(MatrixProperties.Norm));
        assertTrue(properties.contains(MatrixProperties.Diag));
        assertTrue(properties.contains(MatrixProperties.Herm));

    }

    /**
     * checks if m7 is not square
     */
    @Test
    public void notSquareTest(){
        //Q^{2,3} m7 = [2 0 ;0 0 ;0 2];
        final MathValueSymbol m7 = symTab.<MathValueSymbol>resolve
                ("symtab.SymtabMatrixProperties.m7", MathValueSymbol.KIND).orElse(null);
        assertNotNull(m7);
        MathMatrixArithmeticValueSymbol symbol = ((MathMatrixArithmeticValueSymbol)m7.getValue());
        ArrayList<MatrixProperties> properties = symbol.getMatrixProperties();
        assertTrue(m7.getValue() instanceof MathMatrixArithmeticValueSymbol);
        assertFalse(properties.contains(MatrixProperties.Square));
    }

}
