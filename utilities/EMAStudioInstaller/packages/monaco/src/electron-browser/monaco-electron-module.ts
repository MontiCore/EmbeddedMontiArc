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
const module = s.module;
const type = s.process.type;

s.module = undefined;
s.process.type = undefined;

export default loadVsRequire(global)
    .then(vsRequire => {
        s.process.browser = true;

        return loadMonaco(vsRequire);
    })
    .then(() => {
        s.module = module;
        s.process.type = type;

        return import("@theia/monaco/lib/browser/monaco-frontend-module");
    })
    .then(module => module.default);
