package de.monticore.lang.gdl;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import de.monticore.lang.gdl._ast.ASTGame;
import de.monticore.lang.gdl._parser.GDLParser;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Optional;
import org.antlr.v4.runtime.RecognitionException;
import org.junit.Test;

public class GDLCoCosTest {
  
  @Test
  public void testGameStateUpdate() throws RecognitionException, IOException {
    Path model = Paths.get("src/test/resources/gdl/cocos/GameStateUpdate.gdl");
    GDLParser parser = GDLMill.parser();
    Optional<ASTGame> gdlDoc = parser.parse(model.toString());

    assertFalse(parser.hasErrors());
    assertTrue(gdlDoc.isPresent());
  }
}