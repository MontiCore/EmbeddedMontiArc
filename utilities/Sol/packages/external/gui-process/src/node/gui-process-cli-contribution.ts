/*
 * (c) https://github.com/MontiCore/monticore
 */
import { CliContribution } from "@theia/core/lib/node";
import { injectable } from "inversify";
import { Argv, Arguments } from "yargs";

@injectable()
export class GUIProcessCLIContribution implements CliContribution {
    public static readonly RESOLUTION: string = "resolution";

    protected resolution: string;

    public configure(configuration: Argv): void {
        configuration.option(GUIProcessCLIContribution.RESOLUTION, {
            description: "Specifies the resolution of the primary display. Format: WIDTHxHEIGHT",
            type: "string",
            nargs: 1,
            requiresArg: true
        });
    }

    public setArguments(args: Arguments): void {
        const arg = args[GUIProcessCLIContribution.RESOLUTION];

        if (arg !== undefined) this.resolution = arg;
        else throw new Error(`${GUIProcessCLIContribution.RESOLUTION} has not been specified.`);
    }

    public getResolution(): string {
        return this.resolution;
    }
}
