package de.monticore.lang.monticar.emam2wasm.wasm;

import java.nio.file.Path;
import org.apache.commons.io.FilenameUtils;

public class WasmJsNameProvider implements WasmNameProvider {

  private static final String JS_FILE_EXTENSION = "js";

  @Override
  public String getName(Path cppFile) {
    return FilenameUtils.getBaseName(cppFile.toString());
  }

  @Override
  public String getFileExtension() {
    return JS_FILE_EXTENSION;
  }
}
