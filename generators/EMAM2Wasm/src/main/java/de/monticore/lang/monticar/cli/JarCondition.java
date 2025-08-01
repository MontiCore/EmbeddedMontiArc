/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cli;

import org.springframework.context.annotation.Condition;
import org.springframework.context.annotation.ConditionContext;
import org.springframework.core.type.AnnotatedTypeMetadata;

public class JarCondition implements Condition {

  @Override
  public boolean matches(ConditionContext context, AnnotatedTypeMetadata metadata) {
    String classJar = JarCondition.class.getResource("JarCondition.class").toString();
    return classJar.startsWith("jar:");
  }
}
