/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cli;

import org.springframework.context.annotation.Condition;
import org.springframework.context.annotation.ConditionContext;
import org.springframework.core.type.AnnotatedTypeMetadata;

public class WindowsCondition implements Condition {

  @Override
  public boolean matches(ConditionContext context, AnnotatedTypeMetadata metadata) {
    return context.getEnvironment().getProperty("os.name").toLowerCase().contains("windows");
  }
}
