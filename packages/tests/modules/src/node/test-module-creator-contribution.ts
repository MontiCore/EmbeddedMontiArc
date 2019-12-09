/*
 * (c) https://github.com/MontiCore/monticore
 */
import { ModuleCreatorContribution, ModuleCreatorRegistry } from "@embeddedmontiarc/sol-runtime-modules/lib/node";
import { injectable } from "inversify";
import { TestModuleCreator } from "./test-module-creator";

@injectable()
export class TestModuleCreatorContribution implements ModuleCreatorContribution {
    public registerModuleCreators(registry: ModuleCreatorRegistry): void {
        registry.registerModuleCreator(TestModuleCreator);
    }
}
