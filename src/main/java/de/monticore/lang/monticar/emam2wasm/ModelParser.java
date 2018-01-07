package de.monticore.lang.monticar.emam2wasm;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEMACompilationUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._ast.ASTEMAMCompilationUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._parser.EmbeddedMontiArcMathParser;
import java.io.IOException;
import java.io.StringReader;
import java.io.UncheckedIOException;
import java.util.Optional;
import java.util.regex.Pattern;
import java.util.stream.Collectors;

public class ModelParser {

  private EmbeddedMontiArcMathParser parser;

  public ModelParser(EmbeddedMontiArcMathParser parser) {
    this.parser = parser;
  }

  public String parsePackage(String model) {
    try {
      ASTEMAMCompilationUnit ast = parse(model);
      return ast.getEMACompilationUnit().getPackage().stream().collect(Collectors.joining("."));
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
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
