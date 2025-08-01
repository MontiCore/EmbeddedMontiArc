/*
 * (c) https://github.com/MontiCore/monticore
 */
import { inject, injectable } from "inversify";
import { DialogServer } from "../../common";
import { WindowService } from "../window/window-service";
import {
    SaveDialogOptions, SaveDialogReturnValue, OpenDialogOptions, OpenDialogReturnValue,  dialog,
    MessageBoxOptions, MessageBoxReturnValue
} from "electron";

@injectable()
export class DialogServerImpl implements DialogServer {
    @inject(WindowService) protected readonly windows: WindowService;

    public async showSaveDialog(options: SaveDialogOptions): Promise<SaveDialogReturnValue> {
        const mainWindow = this.windows.getMainWindow();

        if (mainWindow) return dialog.showSaveDialog(mainWindow, options);
        else return dialog.showSaveDialog(options);
    }

    public async showOpenDialog(options: OpenDialogOptions): Promise<OpenDialogReturnValue> {
        const mainWindow = this.windows.getMainWindow();

        if (mainWindow) return dialog.showOpenDialog(mainWindow, options);
        else return dialog.showOpenDialog(options);
    }

    public async showMessageBox(options: MessageBoxOptions): Promise<MessageBoxReturnValue> {
        const mainWindow = this.windows.getMainWindow();

        if (mainWindow) return dialog.showMessageBox(mainWindow, options);
        else return dialog.showMessageBox(options);
    }
}
