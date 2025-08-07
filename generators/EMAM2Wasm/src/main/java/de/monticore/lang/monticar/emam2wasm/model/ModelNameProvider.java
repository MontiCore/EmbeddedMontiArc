/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emam2wasm.model;

import java.nio.file.Path;
import java.nio.file.Paths;

/**
 * This interface offers methods that can be used to store a model only available as string in the
 * correct location. The correct location is the location where a model will be correctly loaded
 * into a {@link de.monticore.symboltable.Scope}.
 * <p>
 * Usually, this location is {@code basePath/package/modelName.extension}.
 */
public interface ModelNameProvider {

  /**
   * Returns the name of the supplied model.
   *
   * @param model MontiArc model
   * @return model name
   */
  String getName(String model);

  /**
   * Returns the package name of the supplied model.
   *
   * @param model MontiArc model
   * @return package name
   */
  String getPackage(String model);

  /**
   * Returns the file extension that should be used when storing the model to a file.
   *
   * @return model file extension
   */
  String getFileExtension();

  /**
   * Returns a relative path to the file location where the model should be saved.
   * In this default implementation, the path consists of {@code package/modelName.extension}.
   *
   * @param model MontiArc model
   * @return model file location
   */
  default Path getFilePath(String model) {
    String packagePath = getPackage(model).replace('.', '/');
    String fileName = getName(model) + "." + getFileExtension();
    return Paths.get(packagePath).resolve(fileName);
  }
}
