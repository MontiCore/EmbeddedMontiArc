/*
 * (c) https://github.com/MontiCore/monticore
 */
import { BackendApplicationContribution } from "@theia/core/lib/node";
import { injectable } from "inversify";
import { CommonValidatorRegistry } from "../common";

@injectable()
export class BackendValidatorRegistry extends CommonValidatorRegistry implements BackendApplicationContribution {
    public onStart(): void {
        this.provider.getContributions().forEach(contribution => contribution.registerValidators(this));
    }
}
