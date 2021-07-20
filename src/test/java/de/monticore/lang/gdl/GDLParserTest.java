package de.monticore.lang.gdl;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import de.monticore.lang.gdl._parser.GDLParser;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Optional;
import org.antlr.v4.runtime.RecognitionException;
import org.junit.Test;

public class GDLToolTest {
  
  @Test
  public void testComment() throws RecognitionException, IOException {
    Path model = Paths.get("src/test/resources/gdl/parser/Comment.gdl");
    GDLTool parser = new GDLTool();
    GDLParser parser = GDLMill.parser();
    Optional<ASTGame> gdlDoc = parser.parse(model);

    assertFalse(parser.hasErrors());
    assertTrue(gdlDoc.isPresent());
  }
}