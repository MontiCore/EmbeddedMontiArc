/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cli;

import org.springframework.context.annotation.ConditionContext;
import org.springframework.core.type.AnnotatedTypeMetadata;

public class OtherOSCondition extends WindowsCondition {

  @Override
  public boolean matches(ConditionContext context, AnnotatedTypeMetadata metadata) {
    return !super.matches(context, metadata);
  }
}
