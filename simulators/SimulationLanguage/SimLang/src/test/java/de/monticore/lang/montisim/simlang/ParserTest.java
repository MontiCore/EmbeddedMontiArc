/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang;

import de.monticore.lang.montisim.simlang._ast.ASTSimLangCompilationUnit;
import de.monticore.lang.montisim.simlang._parser.SimLangParser;
import de.monticore.lang.montisim.simlang._symboltable.SimLangLang;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.Test;

import java.io.IOException;
import java.nio.file.FileVisitResult;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.SimpleFileVisitor;
import java.nio.file.attribute.BasicFileAttributes;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import static org.junit.Assert.assertTrue;

public class ParserTest {

  @Before
  public void setUp() {
    // ensure an empty log
    Log.getFindings().clear();
    Log.enableFailQuick(false);
  }


  @Test
  public void testInvalidModels() throws Exception {
    test("sim", "src/test/resources/test/parser", true);
    if (Log.getErrorCount() > 0) {
      throw new Exception("Test Failed, found errors");
    }
  }

  private void test(String fileEnding, String path, boolean expectFailure) throws IOException {
    ParseTest parserTest = new ParseTest("." + SimLangLang.FILE_ENDING, expectFailure);
    Files.walkFileTree(Paths.get(path), parserTest);
    if (!parserTest.getErrors().isEmpty()) {
      Log.debug("Models in error", "ParserTest");
      for (String model : parserTest.getErrors()) {
        Log.debug("  " + model, "ParserTest");
      }
    }
    Log.info(
            "Count of tested models: " + parserTest.getTestCount(),
            "ParserTest"
    );
    Log.info(
            "Count of correctly parsed models: "
                    + (parserTest.getTestCount() - parserTest.getErrors().size()),
            "ParserTest"
    );
    assertTrue(
            "There were models that could not be parsed",
            parserTest.getErrors().isEmpty()
    );
  }

  /**
   * Visits files of the given file ending and checks whether they are parsable.
   *
   * @see Files#walkFileTree(Path, java.nio.file.FileVisitor)
   */
  private static class ParseTest extends SimpleFileVisitor<Path> {
    private final boolean expectFailure;
    private String fileEnding;
    private List<String> errors = new ArrayList<>();
    private int testCount = 0;

    public ParseTest(String fileEnding, boolean expectFailure) {
      super();
      this.fileEnding = fileEnding;
      this.expectFailure = expectFailure;
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
    public List<String> getErrors() {
      return this.errors;
    }

    @Override
    public FileVisitResult visitFile(Path file, BasicFileAttributes attrs) throws IOException {
      if (file.toFile().isFile() && (file.toString().toLowerCase().endsWith(fileEnding))) {
        Log.debug("Parsing file " + file.toString(), "ParserTest");
        testCount++;
        SimLangParser parser = new SimLangParser();
        Optional<ASTSimLangCompilationUnit> ast = parser.parse(file.toString());
        if ((parser.hasErrors() || !ast.isPresent()) && !expectFailure) {
          errors.add(file.toString());
          Log.error("There were unexpected parser errors");
        } else {
          Log.getFindings().clear();
        }
      }
      return FileVisitResult.CONTINUE;
    }
  }
}
