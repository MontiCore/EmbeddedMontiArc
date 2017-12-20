package de.monticore.lang.monticar.emscripten;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

/**
 * This class helps building emscripten compile commands. Parameters are typed as much as possible.
 * However, they are by no means complete.
 */
public class EmscriptenCommandBuilder implements CommandBuilder {

  private final Path emscripten;
  private final Path file;
  private final List<Path> includes = new ArrayList<>();
  private final List<Option> options = new ArrayList<>();
  private Optimization optimizationLevel;
  private boolean bind;
  private String std;
  private String outputFile;

  /**
   * Creates a new builder object. Call {@link #toList()} or {@link #toString()} to build the final
   * command.
   *
   * @param emscripten Path pointing to emscripten binary, e.g.
   * @param file main C++ class
   */
  public EmscriptenCommandBuilder(Path emscripten, Path file) {
    this.emscripten = emscripten;
    this.file = file;
  }

  /**
   * Tells emscripten to include the given path during compilation. The actual command will look
   * like {@code -I"path"}.
   *
   * @param include directory with additional sources to be included during compilation
   */
  public void include(Path include) {
    includes.add(include);
  }

  /**
   * Adds an {@link Option} to the command. The actual command will look like {@code -s option=1}.
   *
   * @param option compile option
   */
  public void addOption(Option option) {
    options.add(option);
  }

  /**
   * Sets the optimization level. By default, no optimization is performed.
   *
   * @param level optimization level
   * @see Optimization
   */
  public void setOptimization(Optimization level) {
    this.optimizationLevel = level;
  }

  /**
   * Activates embind by adding {@code --bind} switch.
   *
   * @param bind if {@code true}, activates embind
   * @see <a href="https://kripken.github.io/emscripten-site/docs/porting/connecting_cpp_and_javascript/embind.html">https://kripken.github.io/emscripten-site/docs/porting/connecting_cpp_and_javascript/embind.html</a>
   */
  public void setBind(boolean bind) {
    this.bind = bind;
  }

  /**
   * Specifies which compiler to use, e.g. {@code c++11}. The actual command will look like
   * {@code -std="std"}.
   *
   * @param std C/C++ compiler to be used for compilation
   */
  public void setStd(String std) {
    this.std = std;
  }

  /**
   * Specifies the name of the output file. This <b>cannot</b> be a path, it has to be a filename
   * with a filename extension. E.g. "module.js".
   * Therefore, the shell has to be opened at the designated target output path.
   *
   * @param outputFile output filename
   */
  public void setOutput(String outputFile) {
    this.outputFile = outputFile;
  }

  /**
   * Returns a command list with the following order:
   * <ol>
   * <li>emscripten binary</li>
   * <li>C++ file</li>
   * <li>Output file name, if present</li>
   * <li>Additional folders to include for compilation, if present</li>
   * <li>Options, if present</li>
   * <li>Optimization level, if present</li>
   * <li>Bind option, if present</li>
   * <li>std option, if present</li>
   * </ol>
   *
   * @return command list
   */
  @Override
  public List<String> toList() {
    List<String> list = new ArrayList<>();
    list.add(emscripten.toString());
    list.add(file.toString());
    if (outputFile != null) {
      list.add("-o");
      list.add(outputFile);
    }
    includes.stream().map(path -> "-I\"" + path.toString() + "\"").forEach(list::add);
    options.stream().map(Option::toString).forEach(s -> {
      list.add("-s");
      list.add(s);
    });
    if (optimizationLevel != null) {
      list.add(optimizationLevel.toString());
    }
    if (bind) {
      list.add("--bind");
    }
    if (std != null) {
      list.add("-std=" + std);
    }
    return list;
  }

  @Override
  public String toString() {
    return toList().stream().collect(Collectors.joining(" "));
  }
}
