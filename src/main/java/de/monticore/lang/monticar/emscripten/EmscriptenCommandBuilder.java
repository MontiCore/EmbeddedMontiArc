/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emscripten;

import static de.monticore.lang.monticar.contract.Precondition.requiresNotNull;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import java.util.stream.Collectors;

/**
 * This class helps building emscripten compile commands. Parameters are typed as much as possible.
 * However, they are by no means complete. <p>
 * At a minimum, it is required to call {@link #setEmscripten(Emscripten)} and {@link #setFile(Path)}.
 */
public class EmscriptenCommandBuilder implements CommandBuilder {

  private Path referenceDir;
  private Emscripten emscripten;
  private Path file;
  private final List<Path> includes = new ArrayList<>();
  private final List<Path> libraries = new ArrayList<>();
  private final List<Option> options = new ArrayList<>();
  private final List<String> flags = new ArrayList<>();
  private Optimization optimizationLevel;
  private boolean bind;
  private String outputFile;

  private static Path relativizeIfNotAbsulute(Path base, Path other) {
    if (!other.isAbsolute()) {
      return normalize(base).relativize(normalize(other)).normalize();
    } else {
      return other;
    }
  }

  private static Path normalize(Path path) {
    return path.toAbsolutePath().normalize();
  }

  /**
   * Emscripten must be called inside the directory where the output files are supposed to be
   * placed. The supplied directory will be used to have all paths supplied through other setters
   * point to the correct path relative to this output directory path if they are not absolute.
   *
   * @param outputDir directory from which emscripten will be called
   * @return this builder
   */
  public EmscriptenCommandBuilder setReferenceOutputDir(Path outputDir) {
    this.referenceDir = requiresNotNull(outputDir);
    return this;
  }

  /**
   * Sets the command calling emscripten if invoked from a terminal on the current operating
   * system.
   *
   * @param emscripten emscripten command
   */
  public EmscriptenCommandBuilder setEmscripten(Emscripten emscripten) {
    this.emscripten = requiresNotNull(emscripten);
    return this;
  }

  /**
   * Sets the main C++ file that is to be compiled by emscripten.
   *
   * @param file main C++ file
   */
  public EmscriptenCommandBuilder setFile(Path file) {
    this.file = requiresNotNull(file);
    return this;
  }

  /**
   * Tells emscripten to include the given path during compilation. The actual command will look
   * like {@code -I"path"}.
   *
   * @param include directory with additional sources to be included during compilation
   * @return this builder
   */
  public EmscriptenCommandBuilder include(Path include) {
    includes.add(requiresNotNull(include));
    return this;
  }

  /**
   * Tells emscripten to include the given path during compilation. The actual command will look
   * like {@code -L"path"}. Use {@link #addFlag(String)} to define the name of the library.
   *
   * @param library directory with a library to be included during compilation
   * @return this builder
   */
  public EmscriptenCommandBuilder addLibrary(Path library) {
    libraries.add(requiresNotNull(library));
    return this;
  }

  /**
   * Adds an {@link Option} to the command. The actual command will look like {@code -s option=1}.
   *
   * @param option compile option
   * @return this builder
   */
  public EmscriptenCommandBuilder addOption(Option option) {
    options.add(requiresNotNull(option));
    return this;
  }

  /**
   * Adds a command line flag to the emscripten command. The actual command will look like
   * {@code -flag}.
   *
   * @param flag command line flag
   * @return this builder
   */
  public EmscriptenCommandBuilder addFlag(String flag) {
    flags.add(flag);
    return this;
  }

  /**
   * Sets the optimization level. By default, no optimization is performed.
   *
   * @param level optimization level
   * @return this builder
   * @see Optimization
   */
  public EmscriptenCommandBuilder setOptimization(Optimization level) {
    this.optimizationLevel = requiresNotNull(level);
    return this;
  }

  /**
   * Activates embind by adding {@code --bind} switch.
   *
   * @param bind if {@code true}, activates embind
   * @return this builder
   * @see <a href="https://kripken.github.io/emscripten-site/docs/porting/connecting_cpp_and_javascript/embind.html">https://kripken.github.io/emscripten-site/docs/porting/connecting_cpp_and_javascript/embind.html</a>
   */
  public EmscriptenCommandBuilder setBind(boolean bind) {
    this.bind = bind;
    return this;
  }

  /**
   * Specifies the name of the output file. This <b>cannot</b> be a path, it has to be a filename
   * with a filename extension. E.g. "module.js".
   * Therefore, the shell has to be opened at the designated target output path.
   *
   * @param outputFile output filename
   * @return this builder
   */
  public EmscriptenCommandBuilder setOutput(String outputFile) {
    this.outputFile = requiresNotNull(outputFile);
    return this;
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
    checkParameters();

    List<String> options = new ArrayList<>();
    options.add(file());
    options.addAll(outputFile());
    options.addAll(includes());
    options.addAll(libraries());
    options.addAll(options());
    options.addAll(flags());
    if (optimizationLevel != null) {
      options.add(optimizationLevel.toString());
    }
    if (bind) {
      options.add("--bind");
    }
    String[] command = emscripten.getCommand(options.toArray(new String[]{}));

    return Arrays.asList(command);
  }

  @Override
  public String toString() {
    if (emscripten == null || file == null) {
      return "";
    }
    return toList().stream().collect(Collectors.joining(" "));
  }

  private String file() {
    return referenceDir != null ? relativizeIfNotAbsulute(referenceDir, file).toString()
        : file.toString();
  }

  private List<String> outputFile() {
    return Arrays.asList(outputFile != null ? new String[]{"-o", outputFile} : new String[]{});
  }

  private void checkParameters() {
    requiresNotNull(emscripten);
    requiresNotNull(file);
  }

  private List<String> includes() {
    return includes.stream()
        .map(path -> referenceDir != null ? relativizeIfNotAbsulute(referenceDir, path) : path)
        .map(path -> "-I\"" + path.toString() + "\"").collect(Collectors.toList());
  }

  private List<String> libraries() {
    return libraries.stream()
        .map(path -> referenceDir != null ? relativizeIfNotAbsulute(referenceDir, path) : path)
        .map(path -> "-L\"" + path.toString() + "\"").collect(Collectors.toList());
  }

  private List<String> options() {
    List<String> res = new ArrayList<>();
    options.stream().map(Option::toString).forEach(s -> {
      res.add("-s");
      res.add(s);
    });
    return res;
  }

  private List<String> flags() {
    return flags.stream().map(flag -> '-' + flag).collect(Collectors.toList());
  }

  @Override
  public final boolean equals(Object o) {
    if (this == o) {
      return true;
    }
    if (!(o instanceof EmscriptenCommandBuilder)) {
      return false;
    }
    EmscriptenCommandBuilder that = (EmscriptenCommandBuilder) o;
    return bind == that.bind &&
        Objects.equals(referenceDir, that.referenceDir) &&
        Objects.equals(emscripten, that.emscripten) &&
        Objects.equals(file, that.file) &&
        Objects.equals(includes, that.includes) &&
        Objects.equals(libraries, that.libraries) &&
        Objects.equals(options, that.options) &&
        Objects.equals(flags, that.flags) &&
        optimizationLevel == that.optimizationLevel &&
        Objects.equals(outputFile, that.outputFile);
  }

  @Override
  public final int hashCode() {
    return Objects.hash(referenceDir, emscripten, file, includes, libraries, options, flags,
        optimizationLevel, bind, outputFile);
  }
}
