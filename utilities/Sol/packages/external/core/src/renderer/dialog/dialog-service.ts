/*
 * (c) https://github.com/MontiCore/monticore
 */
import { inject, injectable } from "inversify";
import { DialogServer } from "../../common";
import {
    SaveDialogOptions, SaveDialogReturnValue, OpenDialogOptions, OpenDialogReturnValue,
    MessageBoxOptions, MessageBoxReturnValue
} from "electron";

export const DialogService = Symbol("DialogService");
/**
 * An interface to be implemented by classes which implement the functionality of showing different kinds of dialogs.
 */
export interface DialogService {
    /**
     * Shows a save dialog in the current main window.
     * @param options The options to be passed to the resulting dialog.
     * @return A promise which holds the results of the opened dialog.
     */
    showSaveDialog(options: SaveDialogOptions): Promise<SaveDialogReturnValue>;

    /**
     * Shows a open dialog in the current main window.
     * @param options The options to be passed to the resulting dialog.
     * @return A promise which holds the results of the opened dialog.
     */
    showOpenDialog(options: OpenDialogOptions): Promise<OpenDialogReturnValue>;

    /**
     * Shows a message box in the current main window.
     * @param options The options to be passed to the resulting message box.
     * @return A promise which holds the results of the opened dialog.
     */
    showMessageBox(options: MessageBoxOptions): Promise<MessageBoxReturnValue>;
}

@injectable()
export class DialogServiceImpl implements DialogService {
    @inject(DialogServer) protected readonly server: DialogServer;

    public async showSaveDialog(options: SaveDialogOptions): Promise<SaveDialogReturnValue> {
        return this.server.showSaveDialog(options);
    }

    public async showOpenDialog(options: OpenDialogOptions): Promise<OpenDialogReturnValue> {
        return this.server.showOpenDialog(options);
    }

    public showMessageBox(options: MessageBoxOptions): Promise<MessageBoxReturnValue> {
        return this.server.showMessageBox(options);
    }
}
