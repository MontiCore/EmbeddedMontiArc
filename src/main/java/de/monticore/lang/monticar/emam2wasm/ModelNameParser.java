package de.monticore.lang.monticar.emam2wasm;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEMACompilationUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._ast.ASTEMAMCompilationUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._parser.EmbeddedMontiArcMathParser;
import java.io.IOException;
import java.io.StringReader;
import java.io.UncheckedIOException;
import java.util.Optional;

public class ModelNameParser {

  private EmbeddedMontiArcMathParser parser;

  public ModelNameParser(EmbeddedMontiArcMathParser parser) {
    this.parser = parser;
  }

  public String parseModelName(String model) {
    try {
      ASTEMAMCompilationUnit ast = parse(model);
      return ast.getEMACompilationUnit().getComponent().getName();
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }

  private ASTEMAMCompilationUnit parse(String model) throws IOException {
    Optional<ASTEMAMCompilationUnit> ast = parser.<ASTEMACompilationUnit>parse(
        new StringReader(model));
    return ast.get();
  }

}
