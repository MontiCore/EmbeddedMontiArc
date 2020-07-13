package de.rwth.montisim.commons.utils.json;

import java.lang.annotation.*;

@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.TYPE)
public @interface JsonType {
    Type value() default Type.OBJECT;
}