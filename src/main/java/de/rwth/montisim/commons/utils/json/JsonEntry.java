/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.utils.json;

import java.lang.annotation.*;

@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.FIELD)
public @interface JsonEntry {
    String value() default ""; // If default -> use field name
}
