/*
 * (c) https://github.com/MontiCore/monticore
 */
import { ContributionProvider } from "@theia/core";
import { EnvVariable } from "@theia/core/lib/common/env-variables";
import { EnvVariablesServerImpl as BaseEnvVariablesServerImpl } from "@theia/core/lib/node/env-variables";
import { inject, injectable, named, postConstruct } from "inversify";

export const EnvVariablesContribution = Symbol("EnvVariablesContribution");
export interface EnvVariablesContribution {
    registerEnvVariables(registry: EnvVariablesRegistry): void;
}

export const EnvVariablesRegistry = Symbol("EnvVariablesRegistry");
export interface EnvVariablesRegistry {
    registerEnvVariable(env: EnvVariable): void;
    unregisterEnvVariable(name: string): void;
}

@injectable()
export class EnvVariablesServerImpl extends BaseEnvVariablesServerImpl implements EnvVariablesRegistry {
    @inject(ContributionProvider) @named(EnvVariablesContribution)
    protected readonly contributions: ContributionProvider<EnvVariablesContribution>;

    @postConstruct()
    protected init(): void {
        this.contributions.getContributions().forEach(contribution => contribution.registerEnvVariables(this));
    }

    public registerEnvVariable(env: EnvVariable): void {
        if (this.envs[env.name]) {
            throw new Error(`There is already an environmental variable with name '${env.name}'.`);
        } else {
            this.envs[env.name] = env;
            process.env[env.name] = env.value;
        }
    }

    public unregisterEnvVariable(name: string): void {
        if (this.envs[name]) {
            delete this.envs[name];
            delete process.env[name];
        } else {
            console.warn(`There is no environmental variable with name '${name}'.`);
        }
    }
}
