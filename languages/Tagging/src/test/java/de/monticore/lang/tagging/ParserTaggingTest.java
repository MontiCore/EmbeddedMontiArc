/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.tagging;

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

import de.monticore.lang.tagging._ast.ASTTaggingUnit;
import de.monticore.lang.tagging._parser.TaggingParser;
import de.se_rwth.commons.logging.Log;
import org.antlr.v4.runtime.RecognitionException;
import org.junit.BeforeClass;
import org.junit.Test;

/**
 *         is copied from MontiArc4/ParserTaggingTest.java
 */
public class ParserTaggingTest {
  public static final boolean ENABLE_FAIL_QUICK = false; // otherwise JUnit test will not fail
  private static List<String> expectedParseErrorModels = Arrays.asList(
      "src/test/resources/LayoutError.tag")
      .stream().map(s -> Paths.get(s).toString())
      .collect(Collectors.toList());

  @BeforeClass
  public static void setUp() {
    // ensure an empty log
    Log.getFindings().clear();
    Log.enableFailQuick(ENABLE_FAIL_QUICK);
  }

  @Test
  public void testTag() throws RecognitionException, IOException {
    test("tag");
  }

  private void test(String fileEnding) throws IOException {
    ParseTest parserTest = new ParseTest("." + fileEnding);
    Files.walkFileTree(Paths.get("src/test/resources"), parserTest);

    if (!parserTest.getModelsInError().isEmpty()) {
      Log.debug("Models in error", "ParserTaggingTest");
      for (String model : parserTest.getModelsInError()) {
        Log.debug("  " + model, "ParserTaggingTest");
      }
    }
    Log.info("Count of tested models: " + parserTest.getTestCount(), "ParserTaggingTest");
    Log.info("Count of correctly parsed models: "
        + (parserTest.getTestCount() - parserTest.getModelsInError().size()), "ParserTaggingTest");

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

        Log.debug("Parsing file " + file.toString(), "ParserTaggingTest");
        testCount++;
        Optional<ASTTaggingUnit> taggingModel = Optional.empty();
        boolean expectingError = ParserTaggingTest.expectedParseErrorModels.contains(file.toString());

        TaggingParser parser = new TaggingParser();
        try {
          if (expectingError) {
            Log.enableFailQuick(false);
          }
          taggingModel = parser.parse(file.toString());
        }
        catch (Exception e) {
          if (!expectingError) {
            Log.error("Exception during test", e);
          }
        }
        if (!expectingError && (parser.hasErrors() || !taggingModel.isPresent())) {
          modelsInError.add(file.toString());
          Log.error("There were unexpected parser errors");
        }
        else {
          Log.getFindings().clear();
        }
        Log.enableFailQuick(ParserTaggingTest.ENABLE_FAIL_QUICK);
      }
      return FileVisitResult.CONTINUE;
    }
  }

  ;

}
