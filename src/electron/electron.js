// @ts-check
const { isMaster } = require("cluster");
const { resolve } = require("path");
const { fork } = require("child_process");
const { app, BrowserWindow, ipcMain } = require("electron");

/*require('electron-debug')({
    enabled: true
});*/


const windows = [];


function onNewWindow(event, url, frameName, disposition, options) {
    options.show = true;
    options.width = 1024;
    options.height = 728;
    options.title = "EmbeddedMontiArcStudio";
}

function onWindowReadyToShow() {
    this.maximize();
    this.show();
}

function onWindowClosed() {
    const index = windows.indexOf(this);

    if (index !== -1) windows.splice(index, 1);

    if (windows.length === 0) app.exit(0);
}

function onBrowserWindowCreated(event, window) {
    window.setMenu(null);
}

function onAppWindowAllClosed() {
    if (process.platform !== "darwin") app.quit();
}

function onIPCCreateNewWindow(event, url) {
    createNewWindow(url);
}

async function onAppReady() {
    const devMode = process.defaultApp || /node_modules[/]electron[/]/.test(process.execPath);
    const clusterPath = resolve(__dirname, "..", "..", "src-gen", "backend", "main.js");

    if (devMode) await runCluster(clusterPath);
    else forkCluster(clusterPath);
}

function onClusterMessage(message) {
    loadMainWindow(message);
}

function onClusterError(error) {
    console.error(error);
    app.exit(1);
}

function onAppQuit() {
    process.kill(this.pid);
}

function onRelayMessage(event, message) {
    for (window of BrowserWindow.getAllWindows()) {
        window.webContents.send("relay", message);
    }
}


function createNewWindow(url) {
    const hasUrl = !!url;
    const window = new BrowserWindow({
        width: 1024, height: 728, show: hasUrl, title: "EmbeddedMontiArcStudio", darkTheme: true
    });

    if (windows.length === 0) window.webContents.on("new-window", onNewWindow);

    windows.push(window);

    if (hasUrl) window.loadURL(url);
    else window.on("ready-to-show", onWindowReadyToShow.bind(window));

    window.on("closed", onWindowClosed.bind(window));

    return window;
}

function loadMainWindow(port) {
    const mainWindow = createNewWindow();

    //mainWindow.loadURL("file://" + join(__dirname, "frontend", "index.html") + "?port=" + port);
    mainWindow.loadURL(`http://localhost:${port}?port=${port}`);
}

async function runCluster(clusterPath) {
    try {
        const address = await require(clusterPath);

        loadMainWindow(address.port);
    } catch(error) {
        console.error(error);
        app.exit(1);
    }
}

function forkCluster(clusterPath) {
    const cluster = fork(clusterPath, [
        "--startup-timeout", "-1",
        "--resources-path", process.resourcesPath
    ]);

    cluster.on("message", onClusterMessage);
    cluster.on("error", onClusterError);
    app.on("quit", onAppQuit.bind(cluster));
}

function startApplication() {
    if (process.env.LC_ALL) process.env.LC_ALL = 'C';

    process.env.LC_NUMERIC = 'C';

    if (isMaster) startFrontend();
    else startBackend();
}

function startFrontend() {
    app.on("window-all-closed", onAppWindowAllClosed);
    ipcMain.on("create-new-window", onIPCCreateNewWindow);
    ipcMain.on("relay", onRelayMessage);
    app.on("ready", onAppReady);
    app.on("browser-window-created", onBrowserWindowCreated);
}

function startBackend() {
    require(resolve(__dirname, "..", "..", "src-gen", "backend", "main.js"));
}


startApplication();