/*
 * (c) https://github.com/MontiCore/monticore
 */
export interface OptionType {
    readonly name: string;
    readonly type: string;
    readonly props: { [name: string]: any; }; // tslint:disable-line:no-any
    readonly options?: OptionType[];
}

export interface OptionsContext {
    readonly workspace: string | undefined;
}
