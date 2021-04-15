/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath;

import static org.junit.Assert.assertTrue;

import java.io.IOException;
import java.nio.file.FileVisitResult;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.SimpleFileVisitor;
import java.nio.file.attribute.BasicFileAttributes;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

import de.monticore.antlr4.MCConcreteParser;
import de.monticore.ast.ASTNode;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._parser.EmbeddedMontiArcMathParser;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.Test;

/**
 */
public class ParserTest {

  private MCConcreteParser parser;

  protected MCConcreteParser getParser() {
    return parser;
  }

  protected void setParser(MCConcreteParser parser) {
    this.parser = parser;
  }

  public static final boolean ENABLE_FAIL_QUICK = false;
  private static List<String> expectedParseErrorModels = Arrays.asList(
      "src/test/resources/emam/arc/context/a/CG12false.arc",

      // "component" is a keyword and may not be used as component name
      "src/test/resources/emam/arc/context/a/component.arc",

      // "connect" is a keyword
      "src/test/resources/emam/arc/context/a/S2.arc",

      // TODO we do not support OCL Expressions yet
      "src/test/resources/emam/arc/prettyPrint/example1/StatusControl.arc",

      // TODO we do not support OCL Expressions yet
      "src/test/resources/emam/arc/symtab/ocl/OCLFieldToPort.arc")

      .stream().map(s -> Paths.get(s).toString())
      .collect(Collectors.toList());

  @Before
  public void setUp() {
    // ensure an empty log
    Log.getFindings().clear();
    Log.enableFailQuick(ENABLE_FAIL_QUICK);
    parser = new EmbeddedMontiArcMathParser();
  }

//  @Test
//  public void testArc() throws Exception {
//    test("arcn");
//    if (Log.getErrorCount() > 0) {
//      throw new Exception("Test Failed, found errors");
//    }
//  }

  @Test
  public void testEmbeddedMontiArcMath() throws Exception {
    test("emam");
    if (Log.getErrorCount() > 0) {
      throw new Exception("Test Failed, found errors");
    }
  }

  private void test(String fileEnding) throws IOException {
    ParseTest parserTest = new ParseTest("." + fileEnding, parser);
    Files.walkFileTree(Paths.get("src/test/resources/emam"), parserTest);

    if (!parserTest.getModelsInError().isEmpty()) {
      Log.debug("Models in error", "ParserTest");
      for (String model : parserTest.getModelsInError()) {
        Log.debug("  " + model, "ParserTest");
      }
    }
    Log.info("Count of tested models: " + parserTest.getTestCount(), "ParserTest");
    Log.info("Count of correctly parsed models: "
        + (parserTest.getTestCount() - parserTest.getModelsInError().size()), "ParserTest");

    assertTrue("There were models that could not be parsed", parserTest.getModelsInError()
        .isEmpty());
  }

  /**
   * Visits files of the given file ending and checks whether they are parsable.
   *
   * @see Files#walkFileTree(Path, java.nio.file.FileVisitor)
   */
  private static class ParseTest extends SimpleFileVisitor<Path> {

    private String fileEnding;

    private MCConcreteParser parser;

    private List<String> modelsInError = new ArrayList<>();

    private int testCount = 0;

    public ParseTest(String fileEnding, MCConcreteParser parser) {
      super();
      this.parser = parser;
      this.fileEnding = fileEnding;
    }

    /**
     * @return testCount
     */
    public int getTestCount() {
      return this.testCount;
    }

    /**
     * @return modelsInError
     */
    public List<String> getModelsInError() {
      return this.modelsInError;
    }

    @Override
    public FileVisitResult visitFile(Path file, BasicFileAttributes attrs)
        throws IOException {
      if (file.toFile().isFile()
          && (file.toString().toLowerCase().endsWith(fileEnding))) {

        Log.debug("Parsing file " + file.toString(), "ParserTest");
        testCount++;
        Optional<? extends ASTNode> maModel = Optional.empty();
        boolean expectingError = ParserTest.expectedParseErrorModels.contains(file.toString());

        try {
          if (expectingError) {
            Log.enableFailQuick(false);
          }
          maModel = parser.parse(file.toString());
        }
        catch (Exception e) {
          if (!expectingError) {
            Log.error("Exception during test", e);
          }
        }
        if (!expectingError && (parser.hasErrors() || !maModel.isPresent())) {
          modelsInError.add(file.toString());
          Log.error("There were unexpected parser errors");
        }
        else {
          Log.getFindings().clear();
        }
        Log.enableFailQuick(ParserTest.ENABLE_FAIL_QUICK);
      }
      return FileVisitResult.CONTINUE;
    }
  }
}
