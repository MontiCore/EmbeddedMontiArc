/*
 * (c) https://github.com/MontiCore/monticore
 */
export const GUI_WINDOW_PATH: string = "/services/gui/window";

export const GUIWindowServer = Symbol("GUIWindowServer");
/**
 * An interface to be implemented by classes which implement the necessary functionality to manipulate the window
 * with the currently running GUI applications.
 */
export interface GUIWindowServer {
    /**
     * Shows the window with the currently running GUI applications.
     */
    show(): Promise<void>;
}
