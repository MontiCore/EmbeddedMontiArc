/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emam2wasm.model;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEMACompilationUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._ast.ASTEMAMCompilationUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._parser.EmbeddedMontiArcMathParser;
import java.io.IOException;
import java.io.StringReader;
import java.io.UncheckedIOException;
import java.util.Optional;
import java.util.stream.Collectors;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Component;

@Component
public class EmamModelNameProvider implements ModelNameProvider {

  private static final String EMAM_EXTENSION = "emam";

  private EmbeddedMontiArcMathParser parser;

  @Autowired
  public EmamModelNameProvider(EmbeddedMontiArcMathParser parser) {
    this.parser = parser;
  }

  @Override
  public String getPackage(String model) {
    try {
      ASTEMAMCompilationUnit ast = parse(model);
      return ast.getEMACompilationUnit().getPackage().stream().collect(Collectors.joining("."));
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }

  @Override
  public String getName(String model) {
    try {
      ASTEMAMCompilationUnit ast = parse(model);
      return ast.getEMACompilationUnit().getComponent().getName();
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }

  @Override
  public String getFileExtension() {
    return EMAM_EXTENSION;
  }

  private ASTEMAMCompilationUnit parse(String model) throws IOException {
    Optional<ASTEMAMCompilationUnit> ast = parser.<ASTEMACompilationUnit>parse(
        new StringReader(model));
    return ast.get();
  }

}
