/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.contract;

import static de.monticore.lang.monticar.contract.FilePrecondition.Condition.exists;
import static de.monticore.lang.monticar.contract.FilePrecondition.Condition.isDirectory;
import static de.monticore.lang.monticar.contract.FilePrecondition.Condition.isFile;
import static de.monticore.lang.monticar.contract.FilePrecondition.Condition.isReadable;
import static de.monticore.lang.monticar.contract.FilePrecondition.Condition.isWritable;
import static de.monticore.lang.monticar.contract.FilePrecondition.Condition.notEmpty;
import static de.monticore.lang.monticar.contract.Precondition.requires;

import de.monticore.lang.monticar.contract.Precondition.PreconditionViolationException;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.LinkOption;
import java.nio.file.Path;
import java.util.function.Supplier;

@SuppressWarnings("WeakerAccess")
public class FilePrecondition {

  /**
   * Assert that the file at the supplied {@link Path} is not empty.
   * <p>
   * <p>A {@code Path} is <em>empty</em> if it is {@code null}, not a file or empty.
   *
   * @param path the path to check
   * @param messageSupplier precondition violation message supplier
   * @return the supplied path as a convenience
   * @throws PreconditionViolationException if the file at the supplied path is empty
   */
  public static <T extends Path> T requiresNotEmpty(final T path,
      final Supplier<String> messageSupplier) {
    requires(notEmpty(path), messageSupplier);
    return path;
  }

  /**
   * @see #requiresNotEmpty(Path, Supplier)
   */
  public static <T extends Path> T requiresNotEmpty(final T path) {
    return requiresNotEmpty(path, () -> NOT_EMPTY_FILE_VIOLATION);
  }

  /**
   * Assert that the file at the supplied {@link Path} is writable.
   *
   * @param path the path to check
   * @param messageSupplier precondition violation message supplier
   * @return the supplied path as a convenience
   * @throws PreconditionViolationException if the file at the supplied path is not writable
   * @see Files#isWritable(Path)
   */
  public static <T extends Path> T requiresWritable(final T path,
      final Supplier<String> messageSupplier) {
    requires(isWritable(path), messageSupplier);
    return path;
  }

  /**
   * @see #requiresWritable(Path, Supplier)
   */
  public static <T extends Path> T requiresWritable(final T path) {
    return requiresWritable(path, () -> WRITABLE_FILE_VIOLATION);
  }

  /**
   * Assert that the file at the supplied {@link Path} is readable.
   *
   * @param path the path to check
   * @param messageSupplier precondition violation message supplier
   * @return the supplied path as a convenience
   * @throws PreconditionViolationException if the file at the supplied path is not readable
   * @see Files#isReadable(Path)
   */
  public static <T extends Path> T requiresReadable(final T path,
      final Supplier<String> messageSupplier) {
    requires(isReadable(path), messageSupplier);
    return path;
  }

  /**
   * @see #requiresReadable(Path, Supplier)
   */
  public static <T extends Path> T requiresReadable(final T path) {
    return requiresReadable(path, () -> READABLE_FILE_VIOLATION);
  }

  /**
   * Assert that the file at the supplied {@link Path} is a directory.
   *
   * @param path the path to check
   * @param messageSupplier precondition violation message supplier
   * @return the supplied path as a convenience
   * @throws PreconditionViolationException if the file at the supplied path is not a directory
   * @see Files#isDirectory(Path, LinkOption...)
   */
  public static <T extends Path> T requiresDirectory(final T path,
      final Supplier<String> messageSupplier) {
    requires(isDirectory(path), messageSupplier);
    return path;
  }

  /**
   * @see #requiresDirectory(Path, Supplier)
   */
  public static <T extends Path> T requiresDirectory(final T path) {
    return requiresDirectory(path, () -> DIRECTORY_VIOLATION);
  }

  /**
   * Assert that the file at the supplied {@link Path} is a file.
   *
   * @param path the path to check
   * @param messageSupplier precondition violation message supplier
   * @return the supplied path as a convenience
   * @throws PreconditionViolationException if the file at the supplied path is not a file
   * @see Files#isRegularFile(Path, LinkOption...)
   */
  public static <T extends Path> T requiresFile(final T path,
      final Supplier<String> messageSupplier) {
    requires(isFile(path), messageSupplier);
    return path;
  }

  /**
   * @see #requiresFile(Path, Supplier)
   */
  public static <T extends Path> T requiresFile(final T path) {
    return requiresFile(path, () -> FILE_VIOLATION);
  }

  /**
   * Assert that the file or directory at the supplied {@link Path} exists.
   *
   * @param path the path to check
   * @param messageSupplier precondition violation message supplier
   * @return the supplied path as a convenience
   * @throws PreconditionViolationException if the file or directory at the supplied path does not
   * exist
   * @see Files#exists(Path, LinkOption...)
   */
  public static <T extends Path> T requiresExists(final T path,
      final Supplier<String> messageSupplier) {
    requires(exists(path), messageSupplier);
    return path;
  }

  /**
   * @see #requiresExists(Path, Supplier)
   */
  public static <T extends Path> T requiresExists(final T path) {
    return requiresExists(path, () -> EXISTS_VIOLATION);
  }

  private static final String NOT_EMPTY_FILE_VIOLATION = "Requires path to point to an existing, non empty file.";
  private static final String WRITABLE_FILE_VIOLATION = "Requires path to point to an existing, writable file.";
  private static final String READABLE_FILE_VIOLATION = "Requires path to point to an existing, readable file.";
  private static final String DIRECTORY_VIOLATION = "Requires path to point to an existing directory.";
  private static final String FILE_VIOLATION = "Requires path to point to an existing file.";
  private static final String EXISTS_VIOLATION = "Requires path to point to an existing file or directory.";

  public static final class Condition {

    public static boolean notEmpty(Path path) {
      try {
        return isReadable(path) && Files.lines(path).anyMatch(s -> true);
      } catch (IOException e) {
        return false;
      }
    }

    public static boolean isWritable(Path path) {
      return isFile(path) && Files.isWritable(path);
    }

    public static boolean isReadable(Path path) {
      return isFile(path) && Files.isReadable(path);
    }

    public static boolean isDirectory(Path path) {
      return exists(path) && Files.isDirectory(path);
    }

    public static boolean isFile(Path path) {
      return exists(path) && Files.isRegularFile(path);
    }

    public static boolean exists(Path path) {
      return null != path && Files.exists(path);
    }
  }

}
