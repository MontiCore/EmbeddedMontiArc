/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang;

import de.monticore.lang.montisim.simlang._ast.ASTSimLangCompilationUnit;
import de.monticore.symboltable.resolving.ResolvedSeveralEntriesException;
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
import java.util.NoSuchElementException;

import static org.junit.Assert.assertTrue;

public class CoCoTest {

  @Before
  public void setUp() {
    Log.getFindings().clear();
    Log.enableFailQuick(false);
  }

  @Test
  public void testValidModels() throws IOException {
    CoCoTester tester = new CoCoTester(".sim",false);
    Files.walkFileTree(Paths.get("src/test/resources/test/coco/valid"), tester);
    logEpilogue(tester);
  }

  @Test
  public void testInvalidModels() throws IOException {
    CoCoTester tester = new CoCoTester(".sim",true);
    Files.walkFileTree(Paths.get("src/test/resources/test/coco/invalid"), tester);
    logEpilogue(tester);
  }

  private void logEpilogue(CoCoTester tester) {
    if (!tester.getErrors().isEmpty()) {
      Log.debug("Models in error", CoCoTest.class.getName());
      for (String model : tester.getErrors()) {
        Log.debug("  " + model, CoCoTest.class.getName());
      }
    }
    Log.info(
            "Count of tested models: " + tester.getTestCount(),
            "CoCoTester"
    );
    Log.info(
            "Count of correctly tested models: "
                    + (tester.getTestCount() - tester.getErrors().size()),
            "CoCoTester"
    );
    assertTrue(
            "There were CoCo check failures",
            tester.getErrors().isEmpty()
    );
  }

  private static class CoCoTester extends SimpleFileVisitor<Path> {
    private final String fileEnding;
    private final boolean expectFailure;
    private final List<String> errors = new ArrayList<>();
    private int testCount = 0;

    public CoCoTester(String fileEnding, boolean expectFailure) {
      super();
      this.fileEnding = fileEnding;
      this.expectFailure = expectFailure;
    }

    public int getTestCount() {
      return this.testCount;
    }

    public List<String> getErrors() {
      return this.errors;
    }

    @Override
    public FileVisitResult visitFile(Path file, BasicFileAttributes attrs) throws IOException {
      if (file.toFile().isFile() && (file.toString().toLowerCase().endsWith(fileEnding))) {
        Log.debug("Parsing file " + file.toString(), CoCoTest.class.getName());
        testCount++;
        ASTSimLangCompilationUnit ast = SimLangTool.parse(file.toString());
        Log.getFindings().clear();
        try {
          SimLangTool.createSymbolTable(SimLangTool.SIMLANG_LANGUAGE, ast);
          SimLangTool.checkDefaultCoCos(ast);
        } catch (NoSuchElementException e) {
          Log.warn("WARNING: This error is only expected in the CoCoTest parsing InfRanges.", e);
        } catch (ResolvedSeveralEntriesException e) {
          Log.warn("WARNING: This error is only expected in the CoCo Test.", e);
        }
        boolean isSuccess = Log.getFindings().isEmpty();
        boolean isTestFailed = (!isSuccess && !expectFailure) || (isSuccess && expectFailure);
        if (isTestFailed) {
          System.out.println("smth went wrong: "+file.toString());
          errors.add(file.toString());
        }
      }
      return FileVisitResult.CONTINUE;
    }
  }
}

