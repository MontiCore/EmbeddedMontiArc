/*
 * (c) https://github.com/MontiCore/monticore
 */
import { FrontendApplicationContribution } from "@theia/core/lib/browser";
import { injectable } from "inversify";
import { CommonValidatorRegistry } from "../common";

@injectable()
export class FrontendValidatorRegistry extends CommonValidatorRegistry implements FrontendApplicationContribution {
    public onStart(): void {
        this.provider.getContributions().forEach(contribution => contribution.registerValidators(this));
    }
}
