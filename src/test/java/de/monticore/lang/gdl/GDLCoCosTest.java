package de.monticore.lang.gdl;

import static org.junit.Assert.*;

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

    String[] models = {
        "AcceptAdd",
        "AcceptDistinct",
        "AcceptDistinctWithToken",
        "AcceptDistinctWithValueAndToken",
        "AcceptDistinctWithValue",
        "AcceptDiv",
        "AcceptDoesWithToken",
        "AcceptDoesWithValue",
        "AcceptEqual",
        "AcceptFunction",
        "AcceptGoal",
        "AcceptGreater",
        "AcceptInference",
        "AcceptInit",
        "AcceptLegal",
        "AcceptLess",
        "AcceptMod",
        "AcceptMult",
        "AcceptNext",
        "AcceptNot",
        "AcceptNotWithExpression",
        "AcceptNotWithFunction",
        "AcceptNumber",
        "AcceptRole",
        "AcceptSub",
        "AcceptSucc",
        "AcceptTerminal",
        "AcceptTrue"
    };
    for(String name:models ){
      Path model = Paths.get("src/test/resources/gdl/cocos/"+name+".gdl");
      GDLParser parser = GDLMill.parser();
      Optional<ASTGame> gdlDoc = parser.parse(model.toString());

      assertFalse(parser.hasErrors());
      assertTrue(gdlDoc.isPresent());

      GDLCoCoChecker checker = new GDLCoCoChecker();
      checker.addCoCo(new ASTGameExpressionCoCo());
      checker.checkAll(gdlDoc.get());
      System.out.print(name);
      System.out.println(Log.getErrorCount());
      assertTrue(Log.getErrorCount() == 0);
    }
  }

  @Test
  public void testFail() throws RecognitionException, IOException {

    String[] models = {
        "FailDistinctWithNoArguments",
        "FailDistinctWithTooFewArguments",
        //Wird unterstützt
        //"FailDistinctWithTooManyArguments",
        "FailDistinctWithWrongFirstArgumentType",
        "FailDistinctWithWrongSecondArgumentType",
        "FailDistinctWithMultipleWrongArgumentTypes",
        "FailDoesWithTooFewArguments",
        "FailDoesWithTooManyArguments",
        "FailDoesWithWrongArgumentTypeOfSecondArgument",
        "FailDoesWithWrongFirstArgumentType",
        "FailDoesWithWrongSecondArgumentType",
        "FailFunction",
        "FailGoalWithNoArguments",
        "FailGoalWithTooFewArguments",
        "FailGoalWithTooManyArguments",
        "FailGoalWithWrongFirstArgumentType",
        "FailGoalWithWrongSecondArgumentType",
        "FailInferenceWithNoArguments",
        "FailInferenceWithTooFewArguments",
        "FailInferenceWithWrongArgumentType",
        "FailInitWithTooFewArguments",
        "FailInitWithTooManyArguments",
        "FailInitWithWrongArgumentType",
        "FailLegalWithTooFewArguments",
        "FailLegalWithTooManyArguments",
        "FailLegalWithWrongArgumentTypeOfSecondArgument",
        // Wird unterstützt:
        // "FailLegalWithWrongFirstArgumentType",
        "FailLegalWithWrongSecondArgumentType",
        "FailNextWithTooFewArguments",
        "FailNextWithTooManyArguments",
        "FailNextWithWrongArgumentType",
        "FailNextWithWrongArgumentTypeOfArgument",
        "FailNotWithTooFewArguments",
        "FailNotWithTooManyArguments",
        // "FailNotWithWrongArgumentTypeOfFirstArgument",
        "FailNotWithWrongFirstArgumentType",
        "FailRoleWithTooFewArguments",
        "FailRoleWithTooManyArguments",
        // "FailTerminalWithWrongArgumentType",
        "FailTrueWithTooFewArguments",
        "FailTrueWithTooManyArguments",
        "FailTrueWithWrongArgumentType",
        "FailTrueWithWrongArgumentTypeOfArgument"
    };
    for(String name:models ){
      Log.clearFindings();
      System.out.println(name);
      Path model = Paths.get("src/test/resources/gdl/cocos/"+name+".gdl");
      GDLParser parser = GDLMill.parser();
      Optional<ASTGame> gdlDoc = parser.parse(model.toString());

      assertFalse(parser.hasErrors());
      assertTrue(gdlDoc.isPresent());

      GDLCoCoChecker checker = new GDLCoCoChecker();
      checker.addCoCo(new ASTGameExpressionCoCo());
      checker.checkAll(gdlDoc.get());
      System.out.print(name);
      System.out.println(Log.getErrorCount());
      assertTrue(Log.getErrorCount() > 0);
    }
  }
  
}