package de.monticore.lang.monticar.setup.action;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatExceptionOfType;

import de.monticore.lang.monticar.junit.TemporaryDirectoryExtension;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

class SetupActionTest {

  public static final String SUBDIR_NAME = "dir";
  public static final String FILE_NAME = "file.txt";

  @Nested
  class WhenCommandFails {

    @ExtendWith(TemporaryDirectoryExtension.class)
    @Test
    void shouldThrowActionException(Path dir) throws IOException {
      Path subdir = dir.resolve(SUBDIR_NAME);
      Files.createDirectory(subdir);

      List<String[]> commands = new ArrayList<>();
      commands.add(new String[]{"cmd", "/C", "echo", "Hello", ">", SUBDIR_NAME});

      SetupAction action = new SetupAction(dir, commands);
      assertThatExceptionOfType(ActionException.class).isThrownBy(action::execute);
    }
  }

  @Nested
  class WithEchoCommand {

    @Test
    @ExtendWith(TemporaryDirectoryExtension.class)
    void shouldCreateFile(Path dir) {
      List<String[]> commands = new ArrayList<>();
      commands.add(new String[]{"cmd", "/C", "echo", "Hello", ">", FILE_NAME});

      SetupAction action = new SetupAction(dir, commands);
      action.execute();

      assertThat(dir.resolve(FILE_NAME)).exists();
    }
  }

}