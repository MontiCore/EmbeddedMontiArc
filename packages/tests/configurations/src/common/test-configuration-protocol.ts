/*
 * (c) https://github.com/MontiCore/monticore
 */
export interface TestConfigurationOptions {
    readonly name: string;
    readonly path: string;
}

export interface TestConfigurationErrors {
    readonly name: string | undefined;
    readonly path: string | undefined;
}
