/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emscripten;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

/**
 * This factory allows initializing a {@link EmscriptenCommandBuilder} with
 * default parameters.
 *
 * Calls to {@link #getBuilder()} will return a new
 * {@code EmscriptenCommandBuilder} with the preset default values. These
 * default values cannot be changed. Calling a setter of a default value on the
 * {@code EmscriptenCommandBuilder} will result in a
 * {@code UnsupportedOperationException}.
 *
 * If the default values are stored in a {@code Collection}, further elements
 * can still be added.
 */
public class EmscriptenCommandBuilderFactory {

  private Emscripten emscripten;
  private Path file;
  private List<Path> includes = new ArrayList<>();
  private final List<Path> libraries = new ArrayList<>();
  private List<Option> options = new ArrayList<>();
  private List<String> flags = new ArrayList<>();
  private Optimization optimizationLevel;
  private Boolean bind;
  private String outputFile;

  /**
   * Sets the default command calling emscripten if invoked from a terminal on
   * the current operating system.
   *
   * @param emscripten default emscripten command
   * @return this factory
   */
  public EmscriptenCommandBuilderFactory setEmscripten(Emscripten emscripten) {
    this.emscripten = emscripten;
    return this;
  }

  /**
   * Sets the default main C++ file that is to be compiled by emscripten.
   *
   * @param file default main C++ file
   * @return this factory
   */
  public EmscriptenCommandBuilderFactory setFile(Path file) {
    this.file = file;
    return this;
  }

  /**
   * Tells emscripten to include the given path during compilation. The actual
   * command will look like {@code -I"path"}.
   *
   * @param include directory with additional sources to be included during
   * compilation
   * @return this factory
   */
  public EmscriptenCommandBuilderFactory include(Path include) {
    includes.add(include);
    return this;
  }

  /**
   * Tells emscripten to include the given path during compilation. The actual command will look
   * like {@code -L"path"}. Use {@link #addFlag(String)} to define the name of the library.
   *
   * @param library directory with a library to be included during compilation
   * @return this builder
   */
  public EmscriptenCommandBuilderFactory addLibrary(Path library) {
    libraries.add(library);
    return this;
  }

  /**
   * Adds an {@link Option} to the command. The actual command will look like
   * {@code -s option=1}.
   *
   * @param option compile option
   * @return this factory
   */
  public EmscriptenCommandBuilderFactory addOption(Option option) {
    options.add(option);
    return this;
  }

  /**
   * Adds a command line flag to the emscripten command. The actual command will look like
   * {@code -flag}.
   *
   * @param flag command line flag
   * @return this builder
   */
  public EmscriptenCommandBuilderFactory addFlag(String flag) {
    flags.add(flag);
    return this;
  }

  /**
   * Sets the default optimization level.
   *
   * @param level default optimization level
   * @return this factory
   * @see Optimization
   */
  public EmscriptenCommandBuilderFactory setOptimization(Optimization level) {
    this.optimizationLevel = level;
    return this;
  }

  /**
   * Activates embind by adding {@code --bind} switch.
   *
   * @param bind if {@code true}, activates embind
   * @return this factory
   * @see <a href="https://kripken.github.io/emscripten-site/docs/porting/connecting_cpp_and_javascript/embind.html">https://kripken.github.io/emscripten-site/docs/porting/connecting_cpp_and_javascript/embind.html</a>
   */
  public EmscriptenCommandBuilderFactory setBind(boolean bind) {
    this.bind = bind;
    return this;
  }

  /**
   * Specifies the default name of the output file. This <b>cannot</b> be a
   * path, it has to be a filename with a filename extension. E.g. "module.js".
   * Therefore, the shell has to be opened at the designated target output
   * path.
   *
   * @param outputFile default output filename
   * @return this factory
   */
  public EmscriptenCommandBuilderFactory setOutput(String outputFile) {
    this.outputFile = outputFile;
    return this;
  }

  /**
   * Creates a new {@link EmscriptenCommandBuilder} and initializes it with the
   * default parameters specified in this class.
   * If a default value is specified, the corresponding setter in
   * {@code EmscriptenCommandBuilder} should not be called anymore and will
   * throw a {@code UnsupportedOperationException}. If a default value is
   * stored in a {@code Collection}, further elements can still be added to the
   * {@code EmscriptenCommandBuilder} object.
   *
   * @return command builder with default values
   */
  public EmscriptenCommandBuilder getBuilder() {
    return new DefaultValueEmscriptenCommandBuilder(emscripten, file, includes, libraries, options,
        optimizationLevel, bind, outputFile);
  }

  private class DefaultValueEmscriptenCommandBuilder extends EmscriptenCommandBuilder {

    private final boolean emscriptenDefault;
    private final boolean fileDefault;
    private final boolean optimizationDefault;
    private final boolean bindDefault;
    private final boolean outputDefault;

    DefaultValueEmscriptenCommandBuilder(
        Emscripten emscripten,
        Path file,
        List<Path> includes,
        List<Path> libraries,
        List<Option> options,
        Optimization optimizationLevel,
        Boolean bind,
        String outputFile) {
      emscriptenDefault = emscripten != null;
      fileDefault = file != null;
      optimizationDefault = optimizationLevel != null;
      bindDefault = bind != null;
      outputDefault = outputFile != null;

      if (emscriptenDefault) {
        super.setEmscripten(emscripten);
      }
      if (fileDefault) {
        super.setFile(file);
      }
      if (optimizationDefault) {
        super.setOptimization(optimizationLevel);
      }
      if (bindDefault) {
        super.setBind(bind);
      }
      if (outputDefault) {
        super.setOutput(outputFile);
      }

      if (includes != null) {
        includes.forEach(super::include);
      }
      if (libraries != null) {
        libraries.forEach(super::addLibrary);
      }
      if (options != null) {
        options.forEach(super::addOption);
      }
      if (flags != null) {
        flags.forEach(super::addFlag);
      }
    }

    /**
     * Sets the command calling emscripten if invoked from a terminal on the current operating
     * system.
     *
     * @param emscripten emscripten command
     * @throws UnsupportedOperationException if a default value was specified
     */
    @Override
    public EmscriptenCommandBuilder setEmscripten(Emscripten emscripten) {
      if (emscriptenDefault) {
        throw new UnsupportedOperationException("Default parameter cannot be changed.");
      }
      return super.setEmscripten(emscripten);
    }

    /**
     * Sets the main C++ file that is to be compiled by emscripten.
     *
     * @param file main C++ file
     * @throws UnsupportedOperationException if a default value was specified
     */
    @Override
    public EmscriptenCommandBuilder setFile(Path file) {
      if (fileDefault) {
        throw new UnsupportedOperationException("Default parameter cannot be changed.");
      }
      return super.setFile(file);
    }

    /**
     * Sets the optimization level. By default, no optimization is performed.
     *
     * @param level optimization level
     * @return this builder
     * @throws UnsupportedOperationException if a default value was specified
     * @see Optimization
     */
    @Override
    public EmscriptenCommandBuilder setOptimization(Optimization level) {
      if (optimizationDefault) {
        throw new UnsupportedOperationException("Default parameter cannot be changed.");
      }
      return super.setOptimization(level);
    }

    /**
     * Activates embind by adding {@code --bind} switch.
     *
     * @param bind if {@code true}, activates embind
     * @return this builder
     * @throws UnsupportedOperationException if a default value was specified
     * @see <a href="https://kripken.github.io/emscripten-site/docs/porting/connecting_cpp_and_javascript/embind.html">https://kripken.github.io/emscripten-site/docs/porting/connecting_cpp_and_javascript/embind.html</a>
     */
    @Override
    public EmscriptenCommandBuilder setBind(boolean bind) {
      if (bindDefault) {
        throw new UnsupportedOperationException("Default parameter cannot be changed.");
      }
      return super.setBind(bind);
    }

    /**
     * Specifies the name of the output file. This <b>cannot</b> be a path, it has to be a filename
     * with a filename extension. E.g. "module.js".
     * Therefore, the shell has to be opened at the designated target output path.
     *
     * @param outputFile output filename
     * @return this builder
     * @throws UnsupportedOperationException if a default value was specified
     */
    @Override
    public EmscriptenCommandBuilder setOutput(String outputFile) {
      if (outputDefault) {
        throw new UnsupportedOperationException("Default parameter cannot be changed.");
      }
      return super.setOutput(outputFile);
    }

  }
}
