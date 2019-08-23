/* (c) https://github.com/MontiCore/monticore */

const { app, BrowserWindow } = require("electron");
const { posix } = require("path");

const url = require("url");

let window = null;

function createWindow () {
    window = new BrowserWindow({
        width: 1280,
        height: 720,
        frame: false,
        resizable: false,
        title: "EMAStudioInstaller Manager",
        icon: posix.join(__dirname, "icon.png")
    });

    window.loadURL(url.format({
        protocol: "file",
        slashes: true,
        pathname: posix.join(__dirname, "index.html")
    }));
    window.webContents.openDevTools({ mode: "detach" });
    window.on("closed", () => { window = null; });
}

app.on("ready", createWindow);

app.on("window-all-closed", () => {
    if (process.platform !== "darwin") app.quit()
});

app.on("activate", () => {
    if (window === null) createWindow();
});
