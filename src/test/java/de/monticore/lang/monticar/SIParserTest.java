/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar;

import de.monticore.lang.monticar.ranges._ast.ASTRange;
import de.monticore.lang.monticar.types2._ast.ASTElementType;
import de.monticore.lang.monticar.types2._parser.Types2Parser;
import de.se_rwth.commons.logging.Log;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import java.io.IOException;

import static org.junit.Assert.assertNotNull;

/**
 */
public class SIParserTest {

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

    /*
        @Test
        public void testM2() throws IOException {
            SIParser parser = new SIParser();
            ASTNumberWithUnit ast = parser.parseString_NumberWithUnit("-0.9").orElse(null);
            assertNotNull(ast);

            assertEquals(Rational.valueOf(-9, 10), ast.getNumber());
            assertEquals(Unit.ONE, ast.getUnit());
        }

        @Test
        public void testM1() throws IOException {
            SIParser parser = new SIParser();
            ASTNumberWithUnit ast = parser.parseString_NumberWithUnit("-0.5 kg*m^2/s^3").orElse(null);
            assertNotNull(ast);

            assertEquals(Rational.valueOf(-1, 2), ast.getNumber());
            assertEquals(Unit.valueOf("kg*m^2/s^3"), ast.getUnit());
        }

        @Test
        public void test0() throws IOException {
            SIParser parser = new SIParser();
            ASTNumberWithUnit ast = parser.parseString_NumberWithUnit("8/3 kg*m^2/s^3").orElse(null);
            assertNotNull(ast);

            assertEquals(Rational.valueOf(8, 3), ast.getNumber());
            assertEquals(Unit.valueOf("kg*m^2/s^3"), ast.getUnit());
        }

        @Test
        public void test1() throws IOException {
            SIParser parser = new SIParser();
            ASTNumberWithUnit ast = parser.parseString_NumberWithUnit("8 kg*m^2/s^3").orElse(null);
            assertNotNull(ast);

            assertEquals(Rational.valueOf(8, 1), ast.getNumber());
            assertEquals(Unit.valueOf("kg*m^2/s^3"), ast.getUnit());
        }

        // "x = m/s" is an expression and should not be parsed as unit number
        @Ignore
        @Test
        public void test2() {
            SIParser parser = new SIParser();
            try {
                parser.parseString_NumberWithUnit("m/s");
            } catch (Exception e) {
            } finally {
                // need to have a parser recognition error on the token TNumberWithUnit
                assertTrue(Log.getErrorCount() > 0);
            }
        }

        //
        //@Ignore
        @Test
        public void testE2() {
            SIParser parser = new SIParser();
            try {
                parser.parseString_NumberWithUnit("1");
            } catch (Exception e) {
            } finally {
                assertTrue(Log.getErrorCount() == 0);
            }
        }

        // "x = s^2" is an expression and should not be parsed as unit number
        @Ignore
        @Test
        public void test3() {
            SIParser parser = new SIParser();
            try {
                parser.parseString_NumberWithUnit("s^2");
            } catch (Exception e) {
            } finally {
                // need to have a parser recognition error on the token TNumberWithUnit
                assertTrue(Log.getErrorCount() > 0);
            }
        }

        @Test
        public void test4() throws IOException {
            SIParser parser = new SIParser();
            ASTComplexNumber ast = parser.parseString_ComplexNumber("-7/3 -0.5i").orElse(null);
            assertNotNull(ast);

            assertEquals(Rational.valueOf(-7, 3), ast.getReal());
            assertEquals(Rational.valueOf(-1, 2), ast.getImg());
        }

        @Test
        public void test5() throws IOException {
            SIParser parser = new SIParser();
            ASTComplexNumber ast = parser.parseString_ComplexNumber("1-2i").orElse(null);
            assertNotNull(ast);

            assertEquals(Rational.valueOf(1, 1), ast.getReal());
            assertEquals(Rational.valueOf(-2, 1), ast.getImg());
        }

        @Test
        public void test6() throws IOException {
            SIParser parser = new SIParser();
            ASTComplexNumber ast = parser.parseString_ComplexNumber("1 -2i").orElse(null);
            assertNotNull(ast);

            assertEquals(Rational.valueOf(1, 1), ast.getReal());
            assertEquals(Rational.valueOf(-2, 1), ast.getImg());
        }

        @Test
        public void test7() throws IOException {
            SIParser parser = new SIParser();
            ASTComplexNumber ast = parser.parseString_ComplexNumber("1  -  2i").orElse(null);
            assertNotNull(ast);

            assertEquals(Rational.valueOf(1, 1), ast.getReal());
            assertEquals(Rational.valueOf(-2, 1), ast.getImg());
        }

        @Test
        public void test8() throws IOException {
            SIParser parser = new SIParser();
            ASTComplexNumber ast = parser.parseString_ComplexNumber("-7/3 -1/2i").orElse(null);
            assertNotNull(ast);

            assertEquals(Rational.valueOf(-7, 3), ast.getReal());
            assertEquals(Rational.valueOf(-1, 2), ast.getImg());
        }

        @Test
        public void test9() throws IOException {
            SIParser parser = new SIParser();
            ASTComplexNumber ast = parser.parseString_ComplexNumber("-0.5-0.5i").orElse(null);
            assertNotNull(ast);

            assertEquals(Rational.valueOf(-1, 2), ast.getReal());
            assertEquals(Rational.valueOf(-1, 2), ast.getImg());
        }
    */
    @Test
    public void testRange1() throws IOException {
        Types2Parser parser = new Types2Parser();
        ASTRange ast = parser.parse_StringRange("(10 m/s : 2 m/s : 20 m/s)").orElse(null);
        assertNotNull(ast);
    }

    @Test
    public void testRange2() throws IOException {
        Types2Parser parser = new Types2Parser();
        ASTRange ast = parser.parse_StringRange("(10  : 2 m/s : 20 m/s)").orElse(null);
        assertNotNull(ast);
    }

    @Test
    public void testRange3() throws IOException {
        Types2Parser parser = new Types2Parser();
        ASTRange ast = parser.parse_StringRange("(oo  : oo)").orElse(null);
        assertNotNull(ast);
    }

    /**
     * Test to see whether float precision syntax is recognized
     */
    @Test
    public void testRangeFloatPrecision() throws IOException {
        Types2Parser parser = new Types2Parser();
        ASTRange ast = parser.parse_StringRange("(10  : f : 20 m/s)").orElse(null);
        assertNotNull(ast);
    }


    /**
     * Test to see whether double precision syntax is recognized
     */
    @Test
    public void testRangeDoublePrecision() throws IOException {
        Types2Parser parser = new Types2Parser();
        ASTRange ast = parser.parse_StringRange("(10  : d : 20 m/s)").orElse(null);
        assertNotNull(ast);
    }

    /**
     * Tests to see whether missing units are still recognized
     */
    @Test
    public void testRangeMissingUnit1() throws IOException {
        Types2Parser parser = new Types2Parser();
        ASTRange ast = parser.parse_StringRange("(10 : 0.1 : 45 km)").orElse(null);
        assertNotNull(ast);
    }


    @Test
    public void testRangeMissingUnit2() throws IOException {
        Types2Parser parser = new Types2Parser();
        ASTRange ast = parser.parse_StringRange("(10 : 0.1 km : 45 )").orElse(null);
        assertNotNull(ast);
    }


    @Test
    public void testRangeMissingUnit3() throws IOException {
        Types2Parser parser = new Types2Parser();
        ASTRange ast = parser.parse_StringRange("(10 km: 0.1 : 45 )").orElse(null);
        assertNotNull(ast);
    }

    @Test
    public void testRangeDegreeUnit1() throws IOException {
        Types2Parser parser = new Types2Parser();
        ASTElementType ast = parser.parse_StringElementType("(10 deg: 0.1 : 45 )").orElse(null);
        assertNotNull(ast);

    }


    @Test
    public void testRangeDegreeUnit2() throws IOException {
        Types2Parser parser = new Types2Parser();
        ASTElementType ast = parser.parse_StringElementType("(-90°:0.001°:90°)").orElse(null);
        assertNotNull(ast);

    }
/*
    @Test
    public void testRange7() throws IOException {
        Types2Parser parser = new Types2Parser();
        ASTRanges ast = parser.parse_StringRanges("[(0 : 0.05 : 10) (10 : 0.1 : 45 km)]").orElse(null);
        assertNotNull(ast);
    }

    @Test
    public void testRange8() throws IOException {
        Types2Parser parser = new Types2Parser();
        ASTRanges ast = parser.parse_StringRanges("[(0 km : 0.05 m : 10km) (10km : 0.1m : 45 km)]").orElse(null);
        assertNotNull(ast);
    }
*/
}
