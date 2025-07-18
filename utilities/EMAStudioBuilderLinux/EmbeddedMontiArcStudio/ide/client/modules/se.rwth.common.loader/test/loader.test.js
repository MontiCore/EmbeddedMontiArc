/* (c) https://github.com/MontiCore/monticore */
"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
const constants_1 = require("../../se.rwth.common.test/constants");
const loader_tc_1 = require("./loader.tc");
const url = constants_1.Constants.HOST + "/modules/se.rwth.common.loader/test/loader.test.html";
fixture("Loader Test").page(url);
test("[1] Loader.hide", async () => {
    const loader = new loader_tc_1.LoaderTestController();
    await loader.hide();
    await loader.checkHide();
});
test("[2] Loader.show", async () => {
    const loader = new loader_tc_1.LoaderTestController();
    const message = "Warkatu I";
    await loader.hide();
    await loader.show(message);
    await loader.checkShow(message);
});
test("[3] Loader.message", async () => {
    const loader = new loader_tc_1.LoaderTestController();
    const message = "Warkatu II";
    await loader.message(message);
    await loader.checkMessage(message);
});
