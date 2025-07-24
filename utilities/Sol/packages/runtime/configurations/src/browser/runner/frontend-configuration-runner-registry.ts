/*
 * (c) https://github.com/MontiCore/monticore
 */
import { FrontendApplicationContribution } from "@theia/core/lib/browser";
import { injectable } from "inversify";
import { CommonConfigurationRunnerRegistry } from "../../common";

@injectable()
export class FrontendConfigurationRunnerRegistryImpl extends CommonConfigurationRunnerRegistry implements FrontendApplicationContribution {
    public onStart(): void {
        this.contributions.getContributions().forEach(contribution => contribution.registerRunners(this));
    }
}
