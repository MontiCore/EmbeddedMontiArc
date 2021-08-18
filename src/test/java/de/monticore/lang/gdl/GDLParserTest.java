package de.monticore.lang.gdl;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import de.monticore.lang.gdl._ast.ASTGame;
import de.monticore.lang.gdl._parser.GDLParser;
import de.se_rwth.commons.logging.Log;
import de.se_rwth.commons.logging.LogStub;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Optional;
import org.antlr.v4.runtime.RecognitionException;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

public class GDLParserTest {

  @Before
  public void clear() {
    Log.clearFindings();
  }

  @BeforeClass
  public static void beforeClass() throws Exception {
    LogStub.init();
  }
  /**
   * Parser should be successful when parsing a comment.
   */
  @Test
  public void testComment() throws RecognitionException, IOException {
    Path model = Paths.get("src/test/resources/gdl/parser/Comment.gdl");
    GDLParser parser = GDLMill.parser();
    Optional<ASTGame> gdlDoc = parser.parse(model.toString());

    assertFalse(parser.hasErrors());
    assertTrue(gdlDoc.isPresent());
  }

  /**
   * Parser should be successful when parsing a comment with a fragment of broken GDL inside of it.
   */
  @Test
  public void testCommentWithBrokenGDL() throws RecognitionException, IOException {
    Path model = Paths.get("src/test/resources/gdl/parser/CommentWithBrokenGDL.gdl");
    GDLParser parser = GDLMill.parser();
    Optional<ASTGame> gdlDoc = parser.parse(model.toString());

    assertFalse(parser.hasErrors());
    assertTrue(gdlDoc.isPresent());
  }

  // /**
  //  * Parser should fail when parsing a game state update that's missing the outer parenthesis.
  //  */
  @Test
  public void testGameStateUpdateWithoutOuterParens() throws RecognitionException, IOException {
    Path model = Paths.get("src/test/resources/gdl/parser/GameStateUpdateWithoutOuterParens.gdl");
    GDLParser parser = GDLMill.parser();
    Optional<ASTGame> gdlDoc = parser.parse(model.toString());

    assertTrue(parser.hasErrors());
    assertFalse(gdlDoc.isPresent());
  }

  /**
   * Parser should fail when parsing a gdl with a missing closing parenthesis.
   */
  @Test
  public void testMissingClosingParen() throws RecognitionException, IOException {
    Path model = Paths.get("src/test/resources/gdl/parser/MissingClosingParen.gdl");
    GDLParser parser = GDLMill.parser();
    Optional<ASTGame> gdlDoc = parser.parse(model.toString());

    assertTrue(parser.hasErrors());
    assertFalse(gdlDoc.isPresent());
  }

  // /**
  //  * Parser should fail when parsing a gdl with a missing opening parenthesis.
  //  */
  @Test
  public void testMissingOpeningParen() throws RecognitionException, IOException {
    Path model = Paths.get("src/test/resources/gdl/parser/MissingOpeningParen.gdl");
    GDLParser parser = GDLMill.parser();
    Optional<ASTGame> gdlDoc = parser.parse(model.toString());

    assertTrue(parser.hasErrors());
    assertFalse(gdlDoc.isPresent());
  }

  /**
   * Parser should succeed when parsing a game of tic tac toe.
   */
  @Test
  public void testTicTacToe() throws RecognitionException, IOException {
    Path model = Paths.get("src/test/resources/gdl/TicTacToe.gdl");
    GDLParser parser = GDLMill.parser();
    Optional<ASTGame> gdlDoc = parser.parse(model.toString());

    assertFalse(parser.hasErrors());
    assertTrue(gdlDoc.isPresent());
  }

}