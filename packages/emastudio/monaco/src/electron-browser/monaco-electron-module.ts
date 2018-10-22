/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

// tslint:disable:no-any

import { ContainerModule } from "inversify";
import { loadVsRequire, loadMonaco } from "@theia/monaco/lib/browser/monaco-loader";

export { ContainerModule };

const s = <any>self;

export default loadVsRequire(global)
    .then(vsRequire => {
        // workaround monaco-css not understanding the environment
        s.module = undefined;
        // workaround monaco-typescript not understanding the environment
        s.process.browser = true;
        s.process.type = undefined;

        return loadMonaco(vsRequire);
    })
    .then(() => {
        s.process.type = "renderer";

        return import("@theia/monaco/lib/browser/monaco-frontend-module");
    })
    .then(module => module.default);
