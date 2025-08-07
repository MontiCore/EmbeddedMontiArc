/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { FileNavigatorWidget } from "@theia/navigator/lib/browser";
import { createFileNavigatorContainer } from "@theia/navigator/lib/browser/navigator-container";
import { BrowserFileNavigatorWidget } from "./browser-navigator-widget";

export default new ContainerModule((bind, unbind, isBound, rebind) => {
    rebind(FileNavigatorWidget).toDynamicValue(ctx => {
        const child = createFileNavigatorContainer(ctx.container);

        child.rebind(FileNavigatorWidget).to(BrowserFileNavigatorWidget);

        return child.get(FileNavigatorWidget);
    });
});
