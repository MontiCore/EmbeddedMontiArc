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
package de.monticore.lang.math.math;

import org.junit.Ignore;
import org.junit.Test;
import static de.monticore.lang.math.math.SymbolTableTestHelper.createSymbolTableOnInputAndExpectErrorCode;

public class DimensionTest {

    @Test
    public void testDimensionNoNumberPresent() throws Exception {
        createSymbolTableOnInputAndExpectErrorCode("script S Q^n a; end", "0xMATH10");
    }

    @Test
    public void testDimensionUnitless() throws Exception {
        createSymbolTableOnInputAndExpectErrorCode("script S Q^1 cm a; end", "0xMATH11");
    }

    @Test
    public void testDimensionIsComplex() throws Exception {
        createSymbolTableOnInputAndExpectErrorCode("script S Q^1+1i a; end", "0xMATH12");
    }

    @Test
    public void testDimensionIsInfinite() throws Exception {
        createSymbolTableOnInputAndExpectErrorCode("script S Q^oo a; end", "0xMATH12");
    }

    @Test
    public void testDimensionIsNoInteger() throws Exception {
        createSymbolTableOnInputAndExpectErrorCode("script S Q^1.3 a; end", "0xMATH13");
    }

    @Test
    public void testDimensionIsZero() throws Exception {
        createSymbolTableOnInputAndExpectErrorCode("script S Q^0 a; end", "0xMATH14");
    }

    @Test
    public void testDimensionIsNegative() throws Exception {
        createSymbolTableOnInputAndExpectErrorCode("script S Q^-1 a; end", "0xMATH14");
    }

    @Test
    public void testMultipleDimensionIsNegative() throws Exception {
        createSymbolTableOnInputAndExpectErrorCode("script S Q^{-1,-2} a; end", "0xMATH14");
    }

    @Test
    public void testTooManyDimensions() throws Exception {
        createSymbolTableOnInputAndExpectErrorCode("script S Q^{1,2,3} a; end", "0xMATH15");
    }

    @Test
    public void testMultipleDimensionsNoInteger() throws Exception {
        createSymbolTableOnInputAndExpectErrorCode("script S Q^{1.7,2} a; end", "0xMATH13");
    }

    @Test
    public void testMultipleDimensionsComplex() throws Exception {
        createSymbolTableOnInputAndExpectErrorCode("script S Q^{1+5i,2} a; end", "0xMATH12");
    }

    @Ignore("grammar does not support it")
    @Test
    public void testMultipleDimensionsIsEmpty() throws Exception {
        createSymbolTableOnInputAndExpectErrorCode("script S Q^{} a; end", "0xMATH15");
    }
}
