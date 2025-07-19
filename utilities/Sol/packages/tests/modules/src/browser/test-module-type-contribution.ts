/*
 * (c) https://github.com/MontiCore/monticore
 */
import { ModuleTypeContribution, ModuleTypeRegistry } from "@embeddedmontiarc/sol-runtime-modules/lib/browser";
import { injectable } from "inversify";

@injectable()
export class TestModuleTypeContribution implements ModuleTypeContribution {
    public registerModuleTypes(registry: ModuleTypeRegistry): void {
        registry.registerModuleType({
            id: "0",
            label: "Module Type 1",
            category: "EmbeddedMontiArcStudio",
            iconClass: "fa fa-th"
        });
    }
}
