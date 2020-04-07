/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEMACompilationUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._parser.EmbeddedMontiArcMathParser;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.Test;

import java.io.IOException;
import java.nio.file.*;
import java.nio.file.attribute.BasicFileAttributes;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

import static org.junit.Assert.assertTrue;

/**
 */
public class ParserTest {
  public static final boolean ENABLE_FAIL_QUICK = false;
  private static List<String> expectedParseErrorModels = Arrays.asList(
      "src/test/resources/arc/context/a/CG12false.arc",

      // "component" is a keyword and may not be used as component name
      "src/test/resources/arc/context/a/component.arc",

      // "connect" is a keyword
      "src/test/resources/arc/context/a/S2.arc",

      // TODO we do not support OCL Expressions yet
      "src/test/resources/arc/prettyPrint/example1/StatusControl.arc",

      // TODO we do not support OCL Expressions yet
      "src/test/resources/arc/symtab/ocl/OCLFieldToPort.arc")

      .stream().map(s -> Paths.get(s).toString())
      .collect(Collectors.toList());

  @Before
  public void setUp() {
    // ensure an empty log
    Log.getFindings().clear();
    Log.enableFailQuick(ENABLE_FAIL_QUICK);
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
    ParseTest parserTest = new ParseTest("." + fileEnding);
    Files.walkFileTree(Paths.get("src/test/resources"), parserTest);

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
  protected static class ParseTest extends SimpleFileVisitor<Path> {

    private String fileEnding;

    private List<String> modelsInError = new ArrayList<>();

    private int testCount = 0;

    public ParseTest(String fileEnding) {
      super();
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
        Optional<ASTEMACompilationUnit> maModel = Optional.empty();
        boolean expectingError = ParserTest.expectedParseErrorModels.contains(file.toString());

        EmbeddedMontiArcMathParser parser = new EmbeddedMontiArcMathParser();
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
          Log.error("There were unexpected parser errors: "+file);
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
