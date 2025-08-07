/*
 * (c) https://github.com/MontiCore/monticore
 */
import { MaybePromise } from "@theia/core";
import { CliContribution } from "@theia/core/lib/node";
import { injectable } from "inversify";
import { Argv, Arguments } from "yargs";

/**
 * @ignore
 */
@injectable()
export class MessagingCliContribution implements CliContribution {
    public static readonly EXTERNAL_HOST: string = "external-hostname";

    protected host: string;

    public configure(configuration: Argv): void {
        configuration.option(MessagingCliContribution.EXTERNAL_HOST, {
            description: "Specifies the IP and Port of the host machine. Format: IP:Port",
            type: "string",
            nargs: 1
        });
    }

    public setArguments(args: Arguments): MaybePromise<void> {
        const arg = args[MessagingCliContribution.EXTERNAL_HOST];

        if (arg !== undefined) this.host = arg;
        else throw new Error(`${MessagingCliContribution.EXTERNAL_HOST} has not been specified.`);
    }

    public getExternalHost(): string {
        return this.host;
    }
}
