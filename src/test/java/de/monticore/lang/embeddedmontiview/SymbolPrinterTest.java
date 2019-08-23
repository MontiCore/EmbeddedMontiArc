/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiview;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.io.IOException;
import java.util.Arrays;
import java.util.Optional;

import de.monticore.lang.embeddedmontiview.embeddedmontiview._parser.EmbeddedMontiViewParser;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable.ComponentScope;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable.ViewPortSymbol;
import de.monticore.lang.monticar.ranges._ast.ASTRange;
import de.monticore.lang.monticar.si._symboltable.SIUnitRangesSymbolReference;
import org.junit.Test;

public class SymbolPrinterTest {

  @Test
  public void PortPrintSIUnitTest() throws IOException {
    EmbeddedMontiViewParser parser = new EmbeddedMontiViewParser();
    ASTRange astRange = parser.parseString_Range("(-100 m/s^2:100 m/s^200)").get();
    SIUnitRangesSymbolReference siref = new SIUnitRangesSymbolReference("SIUnitRangesType", Arrays.asList(astRange));
    ViewPortSymbol viewPortSymbol = ViewPortSymbol.builder().setDirection(true).setName("port1").setTypeReference(Optional.of(siref)).build();
    viewPortSymbol.setEnclosingScope(new ComponentScope());

    String result = viewPortSymbol.toString();

    //    assertEquals()
    assertFalse(result.contains("SIUnitRangesType"));
    assertTrue(result.replace(" ", "").contains("(-100/1m/s²:100/1m/s²)"));
    System.out.println(result);
  }
}
