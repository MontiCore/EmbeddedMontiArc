package de.monticore.lang.monticar.emam2wasm;

import static de.monticore.lang.monticar.contract.Precondition.requiresNotNull;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.resolver.Resolver;
import de.monticore.lang.monticar.resolver.ResolverFactory;
import de.monticore.lang.monticar.util.TextFile;
import java.nio.file.Path;

/**
 * This class offers methods to save EmbeddedMontiArc models with arbitrary packages and names to
 * the correct files in the correct directory hierarchy.
 */
public class EMAMStep {

  private static final String MODEL_EXTENSION = "emam";

  private final Path emamDir;
  private final ModelParser modelParser;

  /**
   * Creates a new {@code EMAMStep} object.
   * Subsequent calls to {@link #save(String)} or {@link #getFile(String)} will return a
   * {@code Path} relative to the supplied {@code emamDir}.
   *
   * @param emamDir Base path where models will be saved to
   * @param modelParser Used to parse the package and name of EmbeddedMontiArc models
   */
  public EMAMStep(Path emamDir, ModelParser modelParser) {
    this.emamDir = requiresNotNull(emamDir);
    this.modelParser = requiresNotNull(modelParser);
  }

  /**
   * Saves the supplied model in a file relative to {@code emamDir}, specified in
   * {@link #EMAMStep(Path, ModelParser)}. The location will be {@code emamDir/package/Model.emam}.
   * Directories for parts of the package are created if they do not yet exist.
   *
   * @param model The model to save
   * @return the {@code Path} to the saved model
   * @see #getFile(String)
   */
  public Path save(String model) {
    requiresNotNull(model);

    Path modelPath = getFile(model);
    TextFile modelFile = new TextFile(modelPath);
    modelFile.write(model);
    return modelPath;
  }

  /**
   * Returns the file location where the supplied model is saved or would be saved using
   * {@link #save(String)}.
   *
   * @param model The model whose file location will be returned
   * @return the (possibly not yet existing) {@code Path} to the model
   */
  public Path getFile(String model) {
    requiresNotNull(model);

    String packageName = modelParser.parsePackage(model);
    String packagePathString = packageName.replace('.', '/');
    Path packagePath = emamDir.resolve(packagePathString);

    String modelName = modelParser.parseModelName(model);
    return packagePath.resolve(modelName + "." + MODEL_EXTENSION);
  }
}
