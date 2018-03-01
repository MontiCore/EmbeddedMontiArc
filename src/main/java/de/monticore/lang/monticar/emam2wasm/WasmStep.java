package de.monticore.lang.monticar.emam2wasm;

import de.monticore.lang.monticar.emscripten.EmscriptenCommandBuilder;
import java.io.File;
import java.io.IOException;
import java.nio.file.Path;

/**
 * This class helps compile C++ files to WebAssembly using emscripten.
 */
public class WasmStep {

  private final Path wasmDir;
  private final WasmNameProvider nameProvider;
  private ProcessBuilder pb = new ProcessBuilder();

  /**
   * Creates a new {@code WasmStep} object.
   *
   * @param wasmDir Directory, that the WebAssembly code is compiled to. This
   * directory will be created if it does not yet exist.
   * @param nameProvider Provides the name for the WebAssembly file
   */
  public WasmStep(Path wasmDir, WasmNameProvider nameProvider) {
    this.wasmDir = wasmDir;
    this.nameProvider = nameProvider;
  }

  /**
   * Compiles the supplied C++ code to WebAssembly code. The returned path
   * points to the output file specified by a {@link WasmNameProvider}.<p>
   * The following values will be set in the supplied
   * {@link EmscriptenCommandBuilder}:
   * <ul>
   * <li>C++ file: {@link EmscriptenCommandBuilder#setFile(Path)}</li>
   * <li>Output file name:
   * {@link EmscriptenCommandBuilder#setOutput(String)}</li>
   * <li>Output directory as reference:
   * {@link EmscriptenCommandBuilder#setReferenceOutputDir(Path)}</li>
   * </ul>
   * Therefore, the supplied {@code EmscriptenCommandBuilder} needs to at
   * least specify the command to call the emscripten binary.
   *
   * @param commandBuilder command builder that at least specifies the
   * emscripten binary command. C++ file, output file name and output reference
   * directory will be set by this method.
   * @param cppFile C++ file to compile to WebAssembly
   * @return path to the compiled output file
   * @throws WasmCompilerException if the compilation failed
   */
  public Path compile(EmscriptenCommandBuilder commandBuilder, Path cppFile)
      throws WasmCompilerException {
    commandBuilder.setOutput(nameProvider.getName(cppFile) + "." + nameProvider.getFileExtension());
    commandBuilder.setFile(cppFile);
    commandBuilder.setReferenceOutputDir(wasmDir);

    pb.directory(toFile(wasmDir));
    pb.command(commandBuilder.toList());
    pb.inheritIO();

    try {
      pb.start().waitFor();
    } catch (InterruptedException | IOException e) {
      throw new WasmCompilerException(e);
    }

    return wasmDir.resolve(nameProvider.getFilePath(cppFile));
  }

  private File toFile(Path path) {
    return path.normalize().toAbsolutePath().toFile();
  }
}
