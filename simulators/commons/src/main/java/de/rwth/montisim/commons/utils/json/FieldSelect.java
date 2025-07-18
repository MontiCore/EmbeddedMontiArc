/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.utils.json;

import java.lang.annotation.*;

@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.TYPE)
public @interface FieldSelect {
    Select value() default Select.ALL;
}
