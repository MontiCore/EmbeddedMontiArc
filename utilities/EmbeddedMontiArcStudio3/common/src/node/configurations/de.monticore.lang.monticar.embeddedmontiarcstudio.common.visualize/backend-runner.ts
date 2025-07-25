/*
 * (c) https://github.com/MontiCore/monticore
 */
import { ToolFactory } from "@embeddedmontiarc/sol-runtime-artifact/lib/node";
import { FileUri } from "@theia/core/lib/node";
import { inject, injectable } from "inversify";
import { OptionsContext } from "@embeddedmontiarc/sol-runtime-options/lib/common";
import { CancellationToken } from "@theia/core/lib/common";
import { VisualizeBackendRunnerTOP } from "./backend-runner-top";
import { VisualizationEMAMToolFactory } from "@embeddedmontiarcstudio/visualization-emam/lib/node/de.monticore.lang.monticar.generators.visualizationemam/tool-factory";

import { VisualizeOptions } from "../../../common/configurations/de.monticore.lang.monticar.embeddedmontiarcstudio.common.visualize/protocol";

import * as path from "path";

@injectable()
export class VisualizeBackendRunner extends VisualizeBackendRunnerTOP {
    @inject(VisualizationEMAMToolFactory) protected readonly visualize: ToolFactory;

    protected async runGenerate(uuid: string, options: VisualizeOptions, context: OptionsContext, token: CancellationToken): Promise<void> {
        if (context.workspace) {
            const url = FileUri.fsPath(context.workspace);
            const modelPath = path.resolve(url, options.modelPath);
            const outputPath = path.resolve(url, options.outputPath);

            console.log(`context=${context.workspace}`);
            console.log(`--model ${options.model}`);
            console.log(`--modelPath ${modelPath}`);
            console.log(`--outputPath ${outputPath}`);

            const process = this.visualize(`--model "${options.model}" --modelPath "${modelPath}" --outputPath "${outputPath}"`, uuid);

            await this.waitForTermination(process, token);
        }
    }
}
