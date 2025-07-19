/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emam2wasm.wasm;

import de.monticore.lang.monticar.emscripten.EmscriptenCommandBuilder;
import de.monticore.lang.monticar.emscripten.EmscriptenCommandBuilderFactory;
import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Component;

/**
 * This class helps compile C++ files to WebAssembly using emscripten.
 */
@Component
public class WasmStep {

  private static final Logger logger = LoggerFactory.getLogger(WasmStep.class);
  private final EmscriptenCommandBuilderFactory commandBuilderFactory;
  private final Path wasmDir;
  private final WasmNameProvider nameProvider;
  private ProcessBuilder pb = new ProcessBuilder();

  /**
   * Creates a new {@code WasmStep} object.<p>
   *
   * <b>Note:</b>
   * The following values will be set later and should not be configured
   * in the supplied {@link EmscriptenCommandBuilderFactory}:
   * <ul>
   * <li>C++ file: {@link EmscriptenCommandBuilder#setFile(Path)}</li>
   * <li>Output file name:
   * {@link EmscriptenCommandBuilder#setOutput(String)}</li>
   * <li>Output directory as reference:
   * {@link EmscriptenCommandBuilder#setReferenceOutputDir(Path)}</li>
   * </ul>
   * The supplied {@code EmscriptenCommandBuilderFactory} needs to at
   * least specify the command to call the emscripten binary.
   *
   * @param commandBuilderFactory factory configured with default parameters
   * for any subsequently obtained {@link EmscriptenCommandBuilder}
   * @param wasmDir directory, that the WebAssembly code is compiled to. This
   * directory will be created if it does not yet exist.
   * @param nameProvider provides the name for the WebAssembly file
   */
  @Autowired
  public WasmStep(EmscriptenCommandBuilderFactory commandBuilderFactory, Path wasmDir,
      WasmNameProvider nameProvider) {
    this.commandBuilderFactory = commandBuilderFactory;
    this.wasmDir = wasmDir;
    this.nameProvider = nameProvider;
  }

  /**
   * Compiles the supplied C++ code to WebAssembly code. The returned path
   * points to the output file specified by a {@link WasmNameProvider}.
   *
   * @param cppFile C++ file to compile to WebAssembly
   * @return path to the compiled output file
   * @throws WasmCompilerException if the compilation failed
   */
  public Path compile(Path cppFile) throws WasmCompilerException {
    EmscriptenCommandBuilder commandBuilder = commandBuilderFactory.getBuilder();
    commandBuilder.setOutput(nameProvider.getName(cppFile) + "." + nameProvider.getFileExtension());
    commandBuilder.setFile(cppFile);
    commandBuilder.setReferenceOutputDir(wasmDir);

    pb.directory(toFile(wasmDir));
    pb.command(commandBuilder.toList());
    pb.inheritIO();

    try {
      logger.info("Compiling C++ to WASM using command: " + commandBuilder.toString());
      int returnCode = pb.start().waitFor();
      if (returnCode != 0) {
        throw new WasmCompilerException("Compiling to WebAssembly failed with code " + returnCode);
      }
    } catch (InterruptedException | IOException e) {
      throw new WasmCompilerException(e);
    }

    return wasmDir.resolve(nameProvider.getFilePath(cppFile));
  }

  private File toFile(Path path) {
    return path.normalize().toAbsolutePath().toFile();
  }
}
