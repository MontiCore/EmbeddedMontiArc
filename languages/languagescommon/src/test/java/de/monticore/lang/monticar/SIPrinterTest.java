/* (c) https://github.com/MontiCore/monticore */
//package de.monticore.lang.montiarc;
//
//import static org.junit.Assert.assertEquals;
//import static org.junit.Assert.assertNotNull;
//
//import de.monticore.lang.monticar.helper.SIPrinter;
//import de.monticore.lang.monticar.si._ast.ASTEMAUnit;
//import de.monticore.lang.monticar.si._parser.SIParser;
//import org.junit.Test;
//
///**
// * Created by MichaelvonWenckstern on 27.01.2017.
// */
//public class SIPrinterTest {
//  @Test
//  public void testKm() throws Exception {
//    SIParser parser = new SIParser();
//    ASTEMAUnit astUnit = parser.parseString_EMAUnit("km").orElse(null);
//    assertNotNull(astUnit);
//    assertEquals("km", SIPrinter.printUnitAST(astUnit));
//  }
//
//  @Test
//  public void testKmPerHour() throws Exception {
//    SIParser parser = new SIParser();
//    ASTEMAUnit astUnit = parser.parseString_EMAUnit("km/h").orElse(null);
//    assertNotNull(astUnit);
//    assertEquals("km/h", SIPrinter.printUnitAST(astUnit));
//  }
//
//  @Test
//  public void testVoltTimesAmpere() throws Exception {
//    SIParser parser = new SIParser();
//    ASTEMAUnit astUnit = parser.parseString_EMAUnit("V*A").orElse(null);
//    assertNotNull(astUnit);
//    assertEquals("V*A", SIPrinter.printUnitAST(astUnit));
//  }
//
//  @Test
//  public void testSquareMeter() throws Exception {
//    SIParser parser = new SIParser();
//    ASTEMAUnit astUnit = parser.parseString_EMAUnit("m^2").orElse(null);
//    assertNotNull(astUnit);
//    assertEquals("m^2", SIPrinter.printUnitAST(astUnit));
//  }
//
//  @Test
//  public void testKgTimesSquareMeterDividedByCubicSeconds() throws Exception {
//    SIParser parser = new SIParser();
//    ASTEMAUnit astUnit = parser.parseString_EMAUnit("kg*m^2/s^3").orElse(null);
//    assertNotNull(astUnit);
//    assertEquals("kg*m^2/s^3", SIPrinter.printUnitAST(astUnit));
//  }
//
//  @Test
//  public void testAccelerationNegExponent() throws Exception {
//    SIParser parser = new SIParser();
//    ASTEMAUnit astUnit = parser.parseString_EMAUnit("m*s^-2").orElse(null);
//    assertNotNull(astUnit);
//    assertEquals("m*s^-2", SIPrinter.printUnitAST(astUnit));
//  }
//
//  @Test
//  public void testNegExponent() throws Exception {
//    SIParser parser = new SIParser();
//    ASTEMAUnit astUnit = parser.parseString_EMAUnit("m/s^-2").orElse(null);
//    assertNotNull(astUnit);
//    assertEquals("m/s^-2", SIPrinter.printUnitAST(astUnit));
//  }
//}
