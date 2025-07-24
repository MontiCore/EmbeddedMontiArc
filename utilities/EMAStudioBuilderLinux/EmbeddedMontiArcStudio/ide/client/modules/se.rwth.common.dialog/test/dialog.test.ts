/* (c) https://github.com/MontiCore/monticore */
import {Constants} from "../../se.rwth.common.test/constants";
import {DialogTestController} from "./dialog.tc";

const url = Constants.HOST + "/modules/se.rwth.common.dialog/test/dialog.test.html";

fixture("Dialog Test").page(url);

test("[1] Dialog.show", async () => {
    const dialog = new DialogTestController();
    const message = "<i>Italic</i>";
    const type = "warning";
    const buttons = ["YES", "MAYBE", "NO"];

    await dialog.show(message, type, buttons);
    await dialog.checkShow(message, type, buttons);
});

test("[2] Dialog.hide", async () => {
    const dialog = new DialogTestController();
    const message = "<i>Italic</i>";
    const type = "warning";
    const buttons = ["YES", "MAYBE", "NO"];

    await dialog.show(message, type, buttons);
    await dialog.hide();
    await dialog.checkHide();
});

test("[3] Dialog.show with Button Click", async () => {
    const dialog = new DialogTestController();
    const message = "<i>Italic</i>";
    const type = "warning";
    const buttons = ["YES", "MAYBE", "NO"];

    await dialog.show(message, type, buttons);
    await dialog.clickButton(1);
    await dialog.checkHide();
});
