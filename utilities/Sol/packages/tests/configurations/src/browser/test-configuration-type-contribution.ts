/*
 * (c) https://github.com/MontiCore/monticore
 */
import { injectable } from "inversify";

import {
    ConfigurationTypeContribution,
    ConfigurationTypeRegistry
} from "@embeddedmontiarc/sol-runtime-configurations/lib/browser";

@injectable()
export class TestConfigurationTypeContribution implements ConfigurationTypeContribution {
    public registerConfigurationTypes(registry: ConfigurationTypeRegistry): void {
        registry.registerConfigurationType({
            id: "id",
            label: "Name",
            category: "Category",
            iconClass: "fa fa-play",
            options: [{
                name: "name",
                type: "de.monticore.lang.monticar.sol.option.types.String",
                props: {
                    label: "Name",
                    required: true
                }
            }, {
                name: "path",
                type: "de.monticore.lang.monticar.sol.option.types.Path",
                props: {
                    label: "Path",
                    required: true
                }
            }]
        });
    }
}
