/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.junit;

import de.monticore.lang.monticar.util.FileUtils;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.extension.AfterEachCallback;
import org.junit.jupiter.api.extension.ExtensionContext;
import org.junit.jupiter.api.extension.ExtensionContext.Namespace;
import org.junit.jupiter.api.extension.ExtensionContext.Store;
import org.junit.jupiter.api.extension.ParameterContext;
import org.junit.jupiter.api.extension.ParameterResolutionException;
import org.junit.jupiter.api.extension.ParameterResolver;

/**
 * This JUnit 5 extension injects a temporary directory into test methods.
 * This temporary directory is created before the test is executed and deleted immediately after the
 * test is finished.
 * <p>
 * Only one temporary directory per test method is supported. The temporary directory cannot be
 * declared in a constructor. It has to be declared as test method parameter.
 */
public class TemporaryDirectoryExtension implements ParameterResolver, AfterEachCallback {

  private static final String KEY = "tempDirectory";

  @Override
  public boolean supportsParameter(ParameterContext parameterContext,
      ExtensionContext extensionContext) throws ParameterResolutionException {
    return extensionContext.getTestMethod().isPresent()
        && parameterContext.getParameter().getType().equals(Path.class);
  }

  @Override
  public Object resolveParameter(ParameterContext parameterContext,
      ExtensionContext extensionContext) throws ParameterResolutionException {
    return getLocalStore(extensionContext).getOrComputeIfAbsent(KEY,
        key -> createTempDirectory(tempDirName(extensionContext)));
  }

  @Override
  public void afterEach(ExtensionContext context) {
    Path tempDirectory = (Path) getLocalStore(context).get(KEY);
    if (tempDirectory != null) {
      delete(tempDirectory);
    }
  }

  private Store getLocalStore(ExtensionContext context) {
    return context.getStore(localNamespace(context));
  }

  private Namespace localNamespace(ExtensionContext context) {
    return Namespace.create(TemporaryDirectoryExtension.class, context);
  }

  private String tempDirName(ExtensionContext context) {
    return context.getDisplayName().replaceAll("[^\\w]", "");
  }

  private Path createTempDirectory(String name) {
    try {
      return Files.createTempDirectory(name);
    } catch (IOException e) {
      throw new ParameterResolutionException("Could not create temp directory", e);
    }
  }

  private void delete(Path dir) {
    FileUtils.delete(dir);
  }
}
