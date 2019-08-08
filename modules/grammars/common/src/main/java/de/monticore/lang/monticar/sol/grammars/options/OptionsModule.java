/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.grammars.options;

import com.google.inject.AbstractModule;
import com.google.inject.Provides;
import com.google.inject.multibindings.Multibinder;
import de.monticore.lang.monticar.sol.grammars.options._cocos.OptionsCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.options.cocos.*;
import de.monticore.lang.monticar.sol.grammars.options.cocos.types.ListOptionType;
import de.monticore.lang.monticar.sol.grammars.options.cocos.types.OptionType;
import de.monticore.lang.monticar.sol.grammars.options.cocos.types.PathOptionType;
import de.monticore.lang.monticar.sol.grammars.options.cocos.types.StringOptionType;
import de.monticore.lang.monticar.sol.grammars.options.cocos.types.props.LabelProp;
import de.monticore.lang.monticar.sol.grammars.options.cocos.types.props.Prop;
import de.monticore.lang.monticar.sol.grammars.options.cocos.types.props.RequiredProp;
import de.monticore.lang.monticar.sol.grammars.options.visitor.OptionsSerializer;
import de.monticore.lang.monticar.sol.grammars.options.visitor.OptionsSerializerImpl;

import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;

public class OptionsModule extends AbstractModule {
    @Override
    protected void configure() {
        this.addBindings();
        this.addMultiBindings();
    }

    private void addBindings() {
        bind(OptionsSerializer.class).to(OptionsSerializerImpl.class);
    }

    private void addMultiBindings() {
        this.addContextConditions();
        this.addOptionTypes();
        this.addProps();
    }

    private void addContextConditions() {
        Multibinder<OptionCoCo> coCos = Multibinder.newSetBinder(binder(), OptionCoCo.class);

        coCos.addBinding().to(NoDuplicatePropCoCo.class);
        coCos.addBinding().to(PropTypeCoCo.class);
        coCos.addBinding().to(RequiredPropCoCo.class);
        coCos.addBinding().to(SubOptionsCoCo.class);
        coCos.addBinding().to(SupportedPropCoCo.class);
        coCos.addBinding().to(SupportedTypeCoCo.class);
        coCos.addBinding().to(UniqueOptionCoCo.class);
    }

    private void addOptionTypes() {
        Multibinder<OptionType> types = Multibinder.newSetBinder(binder(), OptionType.class);

        types.addBinding().to(ListOptionType.class);
        types.addBinding().to(PathOptionType.class);
        types.addBinding().to(StringOptionType.class);
    }

    private void addProps() {
        Multibinder<Prop> props = Multibinder.newSetBinder(binder(), Prop.class);

        props.addBinding().to(LabelProp.class);
        props.addBinding().to(RequiredProp.class);
    }

    @Provides
    protected OptionsCoCoChecker provideCoCoChecker(Set<OptionCoCo> coCos) throws Exception {
        OptionsCoCoChecker checker = new OptionsCoCoChecker();

        Set<String> errorCodes = coCos.stream().map(OptionCoCo::getErrorCode).collect(Collectors.toSet());

        if (coCos.size() == errorCodes.size()) coCos.forEach(coCo -> coCo.registerTo(checker));
        else throw new Exception("The same error code has been used in more than one context condition.");

        return checker;
    }

    @Provides
    protected Map<String, OptionType> provideOptionTypes(Set<OptionType> types) {
        Map<String, OptionType> map = new HashMap<>();

        types.forEach(type -> map.put(type.getIdentifier(), type));

        return map;
    }

    @Provides
    protected Map<String, Prop> provideProps(Set<Prop> types) {
        Map<String, Prop> map = new HashMap<>();

        types.forEach(prop -> map.put(prop.getIdentifier(), prop));

        return map;
    }
}
