/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { EMAM2WASMMenuContribution } from "./emam2wasm-menu-contribution";
import { EMAM2WASMCommandContribution } from "./emam2wasm-command-contribution";
import { EMAM2WASMConditions } from "./emam2wasm-conditions";
import { CommandContribution } from "@theia/core/lib/common";
import { MenuContribution } from "@theia/core/lib/common";

import "../../src/browser/style/index.css";

export default new ContainerModule(bind => {
    bind(EMAM2WASMConditions).toSelf().inSingletonScope();

    bind(EMAM2WASMCommandContribution).toSelf().inSingletonScope();
    bind(CommandContribution).toDynamicValue(
        ctx => ctx.container.get(EMAM2WASMCommandContribution)
    ).inSingletonScope();

    bind(EMAM2WASMMenuContribution).toSelf().inSingletonScope();
    bind(MenuContribution).toDynamicValue(
        ctx => ctx.container.get(EMAM2WASMMenuContribution)
    ).inSingletonScope();
});
