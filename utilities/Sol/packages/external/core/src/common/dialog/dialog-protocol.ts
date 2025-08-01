/*
 * (c) https://github.com/MontiCore/monticore
 */
import {
    OpenDialogOptions, SaveDialogOptions, SaveDialogReturnValue, OpenDialogReturnValue,
    MessageBoxOptions, MessageBoxReturnValue
} from "electron";

export const DIALOG_PATH: string = "/services/dialog";

export const DialogServer = Symbol("DialogServer");
/**
 * An interface to be implemented by classes which implement the functionality of showing different kinds of dialogs.
 */
export interface DialogServer {
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
