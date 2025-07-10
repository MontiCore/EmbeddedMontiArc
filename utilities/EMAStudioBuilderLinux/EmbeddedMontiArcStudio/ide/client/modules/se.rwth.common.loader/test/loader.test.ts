/* (c) https://github.com/MontiCore/monticore */
import {Constants} from "../../se.rwth.common.test/constants";
import {LoaderTestController} from "./loader.tc";

const url = Constants.HOST + "/modules/se.rwth.common.loader/test/loader.test.html";

fixture("Loader Test").page(url);

test("[1] Loader.hide", async () => {
    const loader = new LoaderTestController();

    await loader.hide();
    await loader.checkHide();
});

test("[2] Loader.show", async () => {
    const loader = new LoaderTestController();
    const message = "Warkatu I";

    await loader.hide();
    await loader.show(message);
    await loader.checkShow(message);
});

test("[3] Loader.message", async () => {
    const loader = new LoaderTestController();
    const message = "Warkatu II";

    await loader.message(message);
    await loader.checkMessage(message);
});
