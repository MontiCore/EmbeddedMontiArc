package de.rwth.montisim.commons.utils.json;

import java.lang.annotation.*;

@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.TYPE)
public @interface JsonTyped {
    String value() default "";
}