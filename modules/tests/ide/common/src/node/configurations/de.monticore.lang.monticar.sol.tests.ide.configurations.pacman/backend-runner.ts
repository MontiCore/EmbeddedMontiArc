/*
 * (c) https://github.com/MontiCore/monticore
 */
import { injectable, inject } from "inversify";
import { OptionsContext } from "@embeddedmontiarc/sol-runtime-options/lib/common";
import { CancellationToken } from "@theia/core/lib/common";
import { PacManOptions } from "../../../common/configurations/de.monticore.lang.monticar.sol.tests.ide.configurations.pacman/protocol";
import { PacManBackendRunnerTOP } from "./backend-runner-top";
import { ToolFactory } from "@embeddedmontiarc/sol-runtime-artifact/lib/node";
import { PingToolFactory } from "@embeddedmontiarc/sol-tests-artifact-ide/lib/node/de.monticore.lang.monticar.sol.tests.artifact.ping/tool-factory";

@injectable()
export class PacManBackendRunner extends PacManBackendRunnerTOP {
    @inject(PingToolFactory) protected readonly pingToolFactory: ToolFactory;

    protected async runValidate(uuid: string, options: PacManOptions, context: OptionsContext, token: CancellationToken): Promise<void> {
        this.pingToolFactory("Example 2", uuid);
    }
}
