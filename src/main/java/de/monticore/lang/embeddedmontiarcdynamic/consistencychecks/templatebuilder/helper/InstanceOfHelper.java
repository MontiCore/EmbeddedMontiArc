package de.monticore.lang.embeddedmontiarcdynamic.consistencychecks.templatebuilder.helper;

import freemarker.ext.util.WrapperTemplateModel;
import freemarker.template.TemplateMethodModelEx;
import freemarker.template.TemplateModelException;

import java.util.List;

/**
 * the InstanceOfHelper returns true if the second object in the list matches the type of the first element, else false
 */
public class InstanceOfHelper implements TemplateMethodModelEx {

    @Override
    public Object exec(List list) throws TemplateModelException {
        if (list.size() != 2) {
            throw new TemplateModelException("Wrong arguments for method 'instanceOf'. Method has two required parameters: object and class");
        } else {
            Object object = ((WrapperTemplateModel) list.get(0)).getWrappedObject();
            Object compareClass = ((WrapperTemplateModel) list.get(1)).getWrappedObject();
            if (!(compareClass instanceof Class)) {
                throw new TemplateModelException("Wrong type of the second parameter. It should be Class. Found: " + compareClass.getClass());
            } else {
                Class c = (Class) compareClass;
                return c.isAssignableFrom(object.getClass());
            }
        }
    }
}
