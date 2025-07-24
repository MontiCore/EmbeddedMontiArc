/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emscripten;

import static de.monticore.lang.monticar.contract.FilePrecondition.requiresReadable;
import static de.monticore.lang.monticar.contract.Precondition.requiresNotNull;

import de.monticore.lang.monticar.util.TextFile;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.stream.Stream;

public class EmscriptenConfigFileParser implements ConfigFileParser {

  private TextFile configFile;

  public EmscriptenConfigFileParser(TextFile configFile) {
    this.configFile = requiresNotNull(configFile);
  }

  public Map<String, String> parseConfigurations() {
    requiresReadable(configFile.getPath());

    Map<String, String> configs = new LinkedHashMap<>();
    try (Stream<String> lines = configFile.lines()) {
      lines.forEach(line -> {
        if (line != null) {
          String[] keyValue = line.split("=", 2);
          if (keyValue.length > 0) {
            String key = keyValue[0].trim();
            String value = null;

            if (keyValue.length > 1) {
              value = removeQuotes(keyValue[1].trim());
            }

            if (!isNullOrEmpty(key)) {
              configs.put(key, value);
            }
          }
        }
      });
    }
    return configs;
  }

  private String removeQuotes(String s) {
    String res = s;
    if (s.length() > 0) {
      char first = s.charAt(0);
      if (first == '\'' || first == '"') {
        res = res.substring(1);
      }
      char last = s.charAt(s.length() - 1);
      if (last == '\'' || last == '"') {
        res = res.substring(0, s.length() - 2);
      }
    }
    return res;
  }

  private boolean isNullOrEmpty(String s) {
    return s == null || s.isEmpty();
  }

}
