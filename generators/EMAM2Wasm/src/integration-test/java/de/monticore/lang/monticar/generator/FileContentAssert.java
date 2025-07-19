/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator;

import de.monticore.lang.monticar.util.TextFile;
import java.nio.file.Path;
import java.util.Arrays;
import org.assertj.core.api.AbstractAssert;

public class FileContentAssert extends AbstractAssert<FileContentAssert, String> {

  public FileContentAssert(String s) {
    super(s, FileContentAssert.class);
  }

  public static FileContentAssert assertThat(String actualFileContent) {
    return new FileContentAssert(actualFileContent);
  }

  private static String eraseWhitespaces(String s) {
    s = s.replaceAll("\\R", " ");
    s = s.replaceAll("\\h+", " ");
    s = s.trim();
    return s;
  }

  public void isEqualToFileContent(Path file) {
    isEqualToFileContent(new TextFile(file));
  }

  public void isEqualToFileContent(TextFile file) {
    isNotNull();

    String expectedFileContent = file.read();

    if (!robustEquals(actual, expectedFileContent)) {
      String actualWithoutWhitespaces = eraseWhitespaces(actual);
      String expectedWithoutWhitespaces = eraseWhitespaces(expectedFileContent);
      failWithMessage(
          "%nAfter replacing whitespaces, tabs and line breaks with a single whitespace:%n"
              + "%nExpecting:%n <%s>%nto be equal to:%n <%s>%n  %s%nbut was not.",
          expectedWithoutWhitespaces, actualWithoutWhitespaces,
          getDiffPositionMarker(expectedWithoutWhitespaces, actualWithoutWhitespaces));
    }
  }

  private String getDiffPositionMarker(String s1, String s2) {
    int diffPosition = firstDiffPosition(s1, s2);
    char[] markerArray = new char[diffPosition + 1];
    Arrays.fill(markerArray, ' ');
    markerArray[markerArray.length - 1] = '^';
    return new String(markerArray);
  }

  private int firstDiffPosition(String s1, String s2) {
    for (int i = 0; i < s1.length(); i++) {
      if (i >= s2.length() || s1.charAt(i) != s2.charAt(i)) {
        return i;
      }
    }
    if (s1.length() < s2.length()) {
      return s1.length();
    }
    return -1;
  }

  private boolean robustEquals(String s1, String s2) {
    int i = 0, j = 0;
    while (i < s1.length() && j < s2.length()) {
      char c1 = s1.charAt(i);
      char c2 = s2.charAt(j);
      if (c1 == c2) {
        i++;
        j++;
      } else if (Character.isWhitespace(c1)) {
        i++;
      } else if (Character.isWhitespace(c2)) {
        j++;
      } else {
        return false;
      }
    }
    return true;
  }


}
