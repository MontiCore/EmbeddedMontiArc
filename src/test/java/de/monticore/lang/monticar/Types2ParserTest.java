package de.monticore.lang.monticar;

import static org.junit.Assert.assertNotNull;

import java.io.IOException;

import de.monticore.lang.monticar.types2._ast.ASTElementType;
import de.monticore.lang.monticar.types2._parser.Types2Parser;
import org.junit.Test;

/**
 * Created by MichaelvonWenckstern on 20.12.2017.
 */
public class Types2ParserTest {
  @Test
  public void testDegreeElementType() throws IOException {
    Types2Parser parser = new Types2Parser();
    ASTElementType ast = parser.parseString_ElementType("Q(-90°:90°)").orElse(null);
    assertNotNull(ast);
  }
}
