/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cli;

import de.monticore.lang.monticar.emscripten.Option;
import java.util.HashSet;
import java.util.Set;
import org.springframework.boot.context.properties.ConfigurationPropertiesBinding;
import org.springframework.core.convert.converter.Converter;
import org.springframework.stereotype.Component;

@Component
@ConfigurationPropertiesBinding
public class OptionConverter implements Converter<String, Option> {

  private static final Set<String> trueValues = new HashSet<>(4);

  private static final Set<String> falseValues = new HashSet<>(4);

  static {
    trueValues.add("true");
    trueValues.add("on");
    trueValues.add("yes");
    trueValues.add("1");

    falseValues.add("false");
    falseValues.add("off");
    falseValues.add("no");
    falseValues.add("0");
  }

  @Override
  @SuppressWarnings("ConstantConditions")
  public Option convert(String source) {
    String[] split = source.split("=", 2);
    if (split.length != 2) {
      throw new IllegalArgumentException("Invalid option '" + source + "'");
    }
    boolean bool = stringToBoolean(split[1]);
    return new Option(split[0], bool);
  }

  private Boolean stringToBoolean(String s) {
    String value = s.trim();
    if ("".equals(value)) {
      return null;
    }
    value = value.toLowerCase();
    if (trueValues.contains(value)) {
      return Boolean.TRUE;
    } else if (falseValues.contains(value)) {
      return Boolean.FALSE;
    } else {
      throw new IllegalArgumentException("Invalid boolean value '" + s + "'");
    }
  }
}
