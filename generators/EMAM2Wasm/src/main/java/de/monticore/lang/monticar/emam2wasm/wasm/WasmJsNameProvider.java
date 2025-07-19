/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emam2wasm.wasm;

import static de.monticore.lang.monticar.contract.Precondition.requiresNotNull;

import java.nio.file.Path;
import org.apache.commons.io.FilenameUtils;
import org.springframework.stereotype.Component;

@Component
public class WasmJsNameProvider implements WasmNameProvider {

  private static final String JS_FILE_EXTENSION = "js";

  @Override
  public String getName(Path cppFile) {
    return FilenameUtils.getBaseName(requiresNotNull(cppFile).toString());
  }

  @Override
  public String getFileExtension() {
    return JS_FILE_EXTENSION;
  }
}
