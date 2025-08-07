/* (c) https://github.com/MontiCore/monticore */
package de.monticore.numberunit;

import de.monticore.numberunit._ast.ASTComplexNumber;
import de.monticore.numberunit._ast.ASTNumberWithUnit;
import de.monticore.numberunit._parser.NumberUnitParser;
import de.se_rwth.commons.logging.Log;
import org.jscience.mathematics.number.Rational;
import org.junit.*;

import javax.measure.unit.Unit;
import java.io.IOException;

import static org.junit.Assert.*;

/**
 * Created by michaelvonwenckstern on 10.02.17.
 */
public class NumberUnitParserTest {

    static boolean failQuick;

    @BeforeClass
    public static void startUp() {
        failQuick = Log.isFailQuickEnabled();
        Log.enableFailQuick(false);
    }

    @AfterClass
    public static void tearDown() {
        Log.enableFailQuick(failQuick);
    }

    @Before
    public void clear() {
        Log.getFindings().clear();
    }

    @Ignore
    @Test
    public void testDegree20() throws IOException {
        NumberUnitParser parser = new NumberUnitParser();
        ASTNumberWithUnit ast = parser.parse_String("7°").orElse(null);
        assertNotNull(ast);
        ast = parser.parse_String("-9°C").orElse(null);
        assertNotNull(ast);
        assertEquals(-9, ast.getNumber().get(), 0);
        assertEquals(true, ast.getUn().getDegCelsiusOpt().isPresent());
    }

    @Test
    public void testM2() throws IOException {
        NumberUnitParser parser = new NumberUnitParser();
        ASTNumberWithUnit ast = parser.parse_StringNumberWithUnit("-0.9").orElse(null);
        assertNotNull(ast);

        assertEquals(-0.9, ast.getNumber().get(), 0);
        assertEquals(false, ast.getUnOpt().isPresent());
    }

    @Test
    public void testM1() throws IOException {
        NumberUnitParser parser = new NumberUnitParser();
        ASTNumberWithUnit ast = parser.parse_StringNumberWithUnit("-0.5 kg*m^2/s^3").orElse(null);
        assertNotNull(ast);
        assertEquals(-0.5, ast.getNumber().get(), 0);
        assertNotNull(ast.getUn().getSIUnit());
    }

    // no literal
//    @Test
//    public void test0() throws IOException {
//        NumberUnitParser parser = new NumberUnitParser();
//        ASTNumberWithUnit ast = parser.parse_StringNumberWithUnit("8/3 kg*m^2/s^3").orElse(null);
//        assertNotNull(ast);
//
//        assertEquals((8 / 3), ast.getNumber().get(), 0);
//        assertNotNull(ast.getUn());
//    }

    @Test
    public void test1() throws IOException {
        NumberUnitParser parser = new NumberUnitParser();
        ASTNumberWithUnit ast = parser.parse_StringNumberWithUnit("8 kg*m^2/s^3").orElse(null);
        assertNotNull(ast);

        assertEquals(8/1, ast.getNumber().get(), 0);
        assertNotNull(ast.getUn());
    }


    @Test
    public void test2() {
        NumberUnitParser parser = new NumberUnitParser();
        try {
            ASTNumberWithUnit ast = parser.parse_StringNumberWithUnit("0.3e-7").get();
        } catch (Exception e) {
        } finally {
            // need to have a parser recognition error on the token TNumberWithUnit
            assertTrue(Log.getErrorCount() == 0);
        }
    }

    @Test
    public void testHex() {
        NumberUnitParser parser = new NumberUnitParser();
        try {
            ASTNumberWithUnit ast = parser.parse_StringNumberWithUnit("0xfff").get();
        } catch (Exception e) {
        } finally {
            // need to have a parser recognition error on the token TNumberWithUnit
            assertTrue(Log.getErrorCount() == 0);
        }
    }

    @Test
    public void testE2() {
        NumberUnitParser parser = new NumberUnitParser();
        try {
            parser.parse_StringNumberWithUnit("5m/s");
        } catch (Exception e) {
        } finally {
            // need to have a parser recognition error on the token TNumberWithUnit
            assertTrue(Log.getErrorCount() == 0);
        }
    }

    @Test
    public void testMixedSiNonOfficial() {
        NumberUnitParser parser = new NumberUnitParser();
        try {
            parser.parse_StringNumberWithUnit("5km/h");
        } catch (Exception e) {
        } finally {
            // need to have a parser recognition error on the token TNumberWithUnit
            assertTrue(Log.getErrorCount() == 0);
        }
    }

    @Ignore
    @Test
    public void test3() {
        NumberUnitParser parser = new NumberUnitParser();
        try {
            ASTNumberWithUnit a = parser.parse_StringNumberWithUnit("3.00e+8").get();
        } catch (Exception e) {
        } finally {
            // need to have a parser recognition error on the token TNumberWithUnit
            assertTrue(Log.getErrorCount() > 0);
        }
    }

    /* Wrong test cases now. Can not parse arithmetic expressions
    @Test
    public void test4() throws IOException {
        NumberUnitParser parser = new NumberUnitParser();
        ASTComplexNumber ast = parser.parse_StringComplexNumber("-7/3 -0.5i").orElse(null);
        assertNotNull(ast);

        assertEquals(Rational.valueOf(-7, 3), ast.getReal());
        assertEquals(Rational.valueOf(-1, 2), ast.getIm());
    }
    */

    @Test
    public void test5() throws IOException {
        NumberUnitParser parser = new NumberUnitParser();
        ASTComplexNumber ast = parser.parse_StringComplexNumber("1-2i").orElse(null);
        assertNotNull(ast);

        assertEquals(1, ast.getRealNumber(), 0);
        assertEquals(-2, ast.getImagineNumber(), 0);
    }

    @Test
    public void test6() throws IOException {
        NumberUnitParser parser = new NumberUnitParser();
        ASTComplexNumber ast = parser.parse_StringComplexNumber("1 -2i").orElse(null);
        assertNotNull(ast);

        assertEquals(1, ast.getRealNumber(), 0);
        assertEquals(-2, ast.getImagineNumber(), 0);
    }

    @Test
    public void test7() throws IOException {
        NumberUnitParser parser = new NumberUnitParser();
        ASTComplexNumber ast = parser.parse_StringComplexNumber("1  -  2i").orElse(null);
        assertNotNull(ast);

        assertEquals(1, ast.getRealNumber(), 0);
        assertEquals(-2, ast.getImagineNumber(), 0);
    }

    /* Wrong test cases now. Can not parse arithmetic expressions
    @Test
    public void test8() throws IOException {
        NumberUnitParser parser = new NumberUnitParser();
        ASTComplexNumber ast = parser.parse_StringComplexNumber("-7/3 -1/2i").orElse(null);
        assertNotNull(ast);

        assertEquals(Rational.valueOf(-7, 3), ast.getReal());
        assertEquals(Rational.valueOf(-1, 2), ast.getIm());
    }
    */

    @Test
    public void test9() throws IOException {
        NumberUnitParser parser = new NumberUnitParser();
        ASTComplexNumber ast = parser.parse_StringComplexNumber("-0.5-0.5i").orElse(null);
        assertNotNull(ast);

        assertEquals(-0.5, ast.getRealNumber(), 0);
        assertEquals(-0.5, ast.getImagineNumber(), 0);
    }


    @Test
    public void testInfinite() throws IOException {
        NumberUnitParser parser = new NumberUnitParser();
        ASTNumberWithUnit ast = parser.parse_StringNumberWithUnit("-oo km/h").orElse(null);
        assertNotNull(ast);
        assertEquals(true, ast.isMinusInfinite());
        assertNotNull(ast.getUn().getSIUnit());
    }

    @Test
    public void testDegree() throws IOException {
        NumberUnitParser parser = new NumberUnitParser();
        ASTNumberWithUnit ast = parser.parse_StringNumberWithUnit("-90 °").orElse(null);
        assertNotNull(ast);
        System.out.println(ast);

    }

    @Test
    public void testDegree2() throws IOException {
        NumberUnitParser parser = new NumberUnitParser();
        ASTNumberWithUnit ast = parser.parse_StringNumberWithUnit("-90 deg").orElse(null);
        assertNotNull(ast);
        System.out.println(ast);

    }

}
