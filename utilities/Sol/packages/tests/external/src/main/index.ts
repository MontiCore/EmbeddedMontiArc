/*
 * (c) https://github.com/MontiCore/monticore
 */
import "reflect-metadata";

import { container } from "@embeddedmontiarc/sol-external-core/lib/common/ioc";
import { Application } from "@embeddedmontiarc/sol-external-core/lib/main/application";
import { app } from "electron";

import * as Express from "express";
import * as Path from "path";

import * as debug from "electron-debug";

if (process.argv.indexOf("--debug") > -1) debug({ showDevTools: true, devToolsMode: "detach" });

async function execute() {
    const renderer = Path.resolve(app.getAppPath(), "lib", "renderer");
    const modules = await Promise.all([
        import("@embeddedmontiarc/sol-external-core/lib/main/core-main-module"),
        import("@embeddedmontiarc/sol-external-preparation/lib/main/preparation-main-module"),
        import("@embeddedmontiarc/sol-external-docker/lib/main/docker-main-module"),
        import("@embeddedmontiarc/sol-external-workspace/lib/main/workspace-main-module"),
        import("@embeddedmontiarc/sol-external-gui-process/lib/main/gui-process-main-module")
    ]);

    modules.forEach(module => container.load(module.default));

    const application = container.get<Application>(Application);

    application.use(Express.static(renderer));
    application.setName("EMAStudio");
    application.setImage("registry.git.rwth-aachen.de/monticore/embeddedmontiarc/utilities/embeddedmontiarcstudio-3:0.0.1");
    app.on("ready", () => application.start());
}

execute().catch(error => console.error(error));
