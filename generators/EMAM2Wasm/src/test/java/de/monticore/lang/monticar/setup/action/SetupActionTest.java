/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.setup.action;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatExceptionOfType;

import de.monticore.lang.monticar.junit.TemporaryDirectoryExtension;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

class SetupActionTest {

  private static final String OS_NAME = System.getProperty("os.name").toLowerCase();
  private static final String SUBDIR_NAME = "dir";
  private static final String FILE_NAME = "file.txt";
  private static final String WINDOWS = "windows";

  private static <T> T[] concat(T[] first, T[] second) {
    T[] result = Arrays.copyOf(first, first.length + second.length);
    System.arraycopy(second, 0, result, first.length, second.length);
    return result;
  }

  private String[] shell() {
    return OS_NAME.startsWith(WINDOWS) ? new String[]{"cmd", "/C"}
        : new String[]{"/bin/bash", "-c"};
  }

  @Nested
  class WhenCommandFails {

    @ExtendWith(TemporaryDirectoryExtension.class)
    @Test
    void shouldThrowActionException(Path dir) throws IOException {
      Path subdir = dir.resolve(SUBDIR_NAME);
      Files.createDirectory(subdir);

      String[] command = concat(shell(), new String[]{"echo Hello > " + SUBDIR_NAME});
      List<String[]> commands = new ArrayList<>();
      commands.add(command);

      SetupAction action = new SetupAction(dir, commands);
      assertThatExceptionOfType(ActionException.class).isThrownBy(action::execute);
    }
  }

  @Nested
  class WithEchoCommand {

    @Test
    @ExtendWith(TemporaryDirectoryExtension.class)
    void shouldCreateFile(Path dir) {
      String[] command = concat(shell(), new String[]{"echo Hello > " + FILE_NAME});
      List<String[]> commands = new ArrayList<>();
      commands.add(command);

      SetupAction action = new SetupAction(dir, commands);
      action.execute();

      assertThat(dir.resolve(FILE_NAME)).exists();
    }
  }

}
