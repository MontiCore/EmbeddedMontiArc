package de.monticore.lang.gdl;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import de.monticore.lang.gdl._ast.ASTGame;
import de.monticore.lang.gdl._cocos.*;
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

public class GDLCoCosTest {

  @Before
  public void clear() {
    Log.clearFindings();
  }

  @BeforeClass
  public static void beforeClass() throws Exception {
    LogStub.init();
  }

  @Test
  public void testAccept() throws RecognitionException, IOException {

    String[] models = [ "AcceptDistinct" ]; 
    for(String name:models ){
      Path model = Paths.get("src/test/resources/gdl/cocos/"+name+"gdl");
      GDLParser parser = GDLMill.parser();
      Optional<ASTGame> gdlDoc = parser.parse(model.toString());

      assertFalse(parser.hasErrors());
      assertTrue(gdlDoc.isPresent());

      GDLCoCoChecker checker = new GDLCoCoChecker();
      checker.addCoCo(new ASTGameExpressionCoCo());
      checker.checkAll(gdlDoc.get());
      assertTrue(Log.getErrorCount() == 0);
    }
  }
  
  // @Test
  // public void testAcceptDoes() throws RecognitionException, IOException {
  //   Path model = Paths.get("src/test/resources/gdl/cocos/AcceptDoes.gdl");
  //   GDLParser parser = GDLMill.parser();
  //   Optional<ASTGame> gdlDoc = parser.parse(model.toString());

  //   assertFalse(parser.hasErrors());
  //   assertTrue(gdlDoc.isPresent());

  //   GDLCoCoChecker checker = new GDLCoCoChecker();
  //   checker.addCoCo(new ASTGameExpressionCoCo());
  //   checker.checkAll(gdlDoc.get());
  // }
  
  // @Test
  // public void testAcceptGoal() throws RecognitionException, IOException {
  //   Path model = Paths.get("src/test/resources/gdl/cocos/AcceptGoal.gdl");
  //   GDLParser parser = GDLMill.parser();
  //   Optional<ASTGame> gdlDoc = parser.parse(model.toString());

  //   assertFalse(parser.hasErrors());
  //   assertTrue(gdlDoc.isPresent());

  //   GDLCoCoChecker checker = new GDLCoCoChecker();
  //   checker.addCoCo(new ASTGameExpressionCoCo());
  //   checker.checkAll(gdlDoc.get());
  // }
  
  // @Test
  // public void testAcceptInit() throws RecognitionException, IOException {
  //   Path model = Paths.get("src/test/resources/gdl/cocos/AcceptInit.gdl");
  //   GDLParser parser = GDLMill.parser();
  //   Optional<ASTGame> gdlDoc = parser.parse(model.toString());

  //   assertFalse(parser.hasErrors());
  //   assertTrue(gdlDoc.isPresent());

  //   GDLCoCoChecker checker = new GDLCoCoChecker();
  //   checker.addCoCo(new ASTGameExpressionCoCo());
  //   checker.checkAll(gdlDoc.get());
  // }
  
  // @Test
  // public void testAcceptLegal() throws RecognitionException, IOException {
  //   Path model = Paths.get("src/test/resources/gdl/cocos/AcceptLegal.gdl");
  //   GDLParser parser = GDLMill.parser();
  //   Optional<ASTGame> gdlDoc = parser.parse(model.toString());

  //   assertFalse(parser.hasErrors());
  //   assertTrue(gdlDoc.isPresent());

  //   GDLCoCoChecker checker = new GDLCoCoChecker();
  //   checker.addCoCo(new ASTGameExpressionCoCo());
  //   checker.checkAll(gdlDoc.get());
  // }
   
  // @Test
  // public void testAcceptNext() throws RecognitionException, IOException {
  //   Path model = Paths.get("src/test/resources/gdl/cocos/AcceptNext.gdl");
  //   GDLParser parser = GDLMill.parser();
  //   Optional<ASTGame> gdlDoc = parser.parse(model.toString());

  //   assertFalse(parser.hasErrors());
  //   assertTrue(gdlDoc.isPresent());

  //   GDLCoCoChecker checker = new GDLCoCoChecker();
  //   checker.addCoCo(new ASTGameExpressionCoCo());
  //   checker.checkAll(gdlDoc.get());
  // }
  
  // @Test
  // public void testAcceptNot() throws RecognitionException, IOException {
  //   Path model = Paths.get("src/test/resources/gdl/cocos/AcceptNot.gdl");
  //   GDLParser parser = GDLMill.parser();
  //   Optional<ASTGame> gdlDoc = parser.parse(model.toString());

  //   assertFalse(parser.hasErrors());
  //   assertTrue(gdlDoc.isPresent());

  //   GDLCoCoChecker checker = new GDLCoCoChecker();
  //   checker.addCoCo(new ASTGameExpressionCoCo());
  //   checker.checkAll(gdlDoc.get());
  // }
  
  // @Test
  // public void testAcceptRole() throws RecognitionException, IOException {
  //   Path model = Paths.get("src/test/resources/gdl/cocos/AcceptRole.gdl");
  //   GDLParser parser = GDLMill.parser();
  //   Optional<ASTGame> gdlDoc = parser.parse(model.toString());

  //   assertFalse(parser.hasErrors());
  //   assertTrue(gdlDoc.isPresent());

  //   GDLCoCoChecker checker = new GDLCoCoChecker();
  //   checker.addCoCo(new ASTGameExpressionCoCo());
  //   checker.checkAll(gdlDoc.get());
  // }
  
  // @Test
  // public void testAcceptTerminal() throws RecognitionException, IOException {
  //   Path model = Paths.get("src/test/resources/gdl/cocos/AcceptTerminal.gdl");
  //   GDLParser parser = GDLMill.parser();
  //   Optional<ASTGame> gdlDoc = parser.parse(model.toString());

  //   assertFalse(parser.hasErrors());
  //   assertTrue(gdlDoc.isPresent());

  //   GDLCoCoChecker checker = new GDLCoCoChecker();
  //   checker.addCoCo(new ASTGameExpressionCoCo());
  //   checker.checkAll(gdlDoc.get());
  // }

  // @Test
  // public void testFailInferenceWithTooFewArguments() throws RecognitionException, IOException {
  //   Path model = Paths.get("src/test/resources/gdl/cocos/FailInferenceWithTooFewArguments.gdl");
  //   GDLParser parser = GDLMill.parser();
  //   Optional<ASTGame> gdlDoc = parser.parse(model.toString());

  //   assertFalse(parser.hasErrors());
  //   assertTrue(gdlDoc.isPresent());
    
  //   GDLCoCoChecker checker = new GDLCoCoChecker();
  //   checker.addCoCo(new ASTGameExpressionCoCo());
  //   checker.checkAll(gdlDoc.get());
  //   assert(Log.getErrorCount() > 0);
  // }
}