package de.rwth.montisim.commons.utils.json;

import java.lang.annotation.*;

@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.TYPE)
public @interface Jsonable {
    StructureType type() default StructureType.OBJECT;
    FieldSelect fields() default FieldSelect.EXPLICIT;
    FieldOptional optional() default FieldOptional.EXPLICIT;
}