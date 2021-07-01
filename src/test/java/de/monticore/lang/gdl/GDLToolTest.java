package de.monticore.lang.gdl;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Optional;

import org.antlr.v4.runtime.RecognitionException;
import org.junit.Test;

import de.monticore.lang.gdl.GDLTool;

public class GDLToolTest {
  
  @Test
  public void testComment() throws RecognitionException, IOException {
    Path model = Paths.get("src/test/resources/gdl/Comment.gdl");
    GDLParser parser = new GDLParser();
    
    Optional<ASTGame> gdlDoc = parser.parse(model.toString());
    assertFalse(parser.hasErrors());
    assertTrue(gdlDoc.isPresent());
  }

}