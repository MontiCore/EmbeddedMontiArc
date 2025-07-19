/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emam2wasm.cpp;

import static de.monticore.lang.monticar.contract.Precondition.requiresNotNull;

import de.monticore.symboltable.Symbol;
import java.nio.file.Path;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Component;

/**
 * This class is used to compile MontiArc models to C++.
 * It combines the functionality of {@link CppCompiler} and {@link CppNameProvider}.
 */
@Component
public class CppStep<T extends Symbol> {

  private final CppCompiler<T> cppCompiler;
  private final Path cppDir;
  private final CppNameProvider nameProvider;

  /**
   * Creates a new {@code CppStep} object.
   *
   * @param cppCompiler MontiArc to C++ compiler
   * @param cppDir Directory, that the C++ code is compiled to. This directory
   * will be created if it does not yet exist.
   * @param nameProvider Provides the name for the main C++ file
   */
  @Autowired
  public CppStep(CppCompiler<T> cppCompiler, Path cppDir, CppNameProvider nameProvider) {
    this.cppCompiler = requiresNotNull(cppCompiler);
    this.cppDir = requiresNotNull(cppDir);
    this.nameProvider = requiresNotNull(nameProvider);
  }

  /**
   * Compiles the supplied MontiArc model to C++ code. The returned path points
   * to the main C++ file that can be used to instantiate the model.
   *
   * @param model MontiArc model
   * @return Path to main C++ file
   * @throws CppCompilerException if something goes wrong during compilation
   */
  public Path compileToCpp(T model) throws CppCompilerException {
    requiresNotNull(model);

    String cppMainName = nameProvider.getName(model) + "." + nameProvider.getFileExtension();
    Path cppFile = cppCompiler.compile(model, cppDir, cppMainName);
    return cppFile;
  }
}
