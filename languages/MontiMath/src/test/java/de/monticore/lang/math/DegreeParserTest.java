/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math;

import static org.junit.Assert.assertNotNull;

import java.io.IOException;

import de.monticore.antlr4.MCConcreteParser;
import de.monticore.lang.math._ast.ASTAssignmentType;
import de.monticore.lang.math._ast.ASTMathDeclarationStatement;
import de.monticore.lang.math._ast.ASTMathStatements;
import de.monticore.lang.math._parser.MathParser;
import de.monticore.lang.monticar.types2._ast.ASTElementType;
import org.junit.Test;

/**
 * Created by MichaelvonWenckstern on 20.12.2017.
 */
public class DegreeParserTest {
  // this test fails right now, but I do not understand why?
  // since the same test directly invoked with Types2 parser works:
  // https://github.com/EmbeddedMontiArc/languagescommon/blob/master/src/test/java/de/monticore/lang/monticar/Types2ParserTest.java#L40
  @Test
  public void testElementType() throws IOException {
    MathParser parser = new MathParser();
    ASTElementType ast = parser.parse_StringElementType("Q(-90°:90°)").orElse(null);
    assertNotNull(ast);
  }

  @Test
  public void testMathDeclarationExpression() throws IOException {
    MathParser parser = new MathParser();
    ASTMathDeclarationStatement ast = parser.parse_StringMathDeclarationStatement("Q(-90°:90°)^{2} x;").orElse(null);
    assertNotNull(ast);
  }
}
