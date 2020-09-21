package de.monticore.lang.embeddedmontiarcdynamic.consistencychecks.templatebuilder.helper;

import freemarker.ext.util.WrapperTemplateModel;
import freemarker.template.TemplateMethodModelEx;
import freemarker.template.TemplateModelException;

import java.util.List;

/**
 * The PortSequenceHelper helps to identify if a port definition of a port of a sequence was already defined.
 * If the port with the name on index 1 in @arguments was not defined before, i.e. not in the list in index 1,
 * the port gets added to the list and the helper returns false, otherwise the helper just returns true.
 */
public class PortSequenceHelper implements TemplateMethodModelEx {

    @Override
    public Object exec(List arguments) throws TemplateModelException {

        List<String> list = (List<String>) ((WrapperTemplateModel) arguments.get(0)).getWrappedObject();
        String name = arguments.get(1).toString();

        boolean contains = list.contains(name);

        if(contains)
            return true;

        list.add(name);

        return false;
    }
}
