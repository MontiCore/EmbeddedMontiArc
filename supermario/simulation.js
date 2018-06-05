var UPDIRECTION    = 0,
    DOWNDIRECTION  = 1,
    LEFTDIRECTION  = 2,
    RIGHTDIRECTION = 3,
    UNIT           = "cm";


function startSimulation () {
    //PACMAN.startNewGame();
    try {
        window.setInterval(doSimulationStep, 500);
    } catch (err) {
        if (err.message === undefined) {
            console.log(err)
        }
        else {
            console.log(err.message);
        }
    }
}

function pressKey(dir) {
    var keyCode;
    if(dir == DOWNDIRECTION)
            keyCode = KEY.ARROW_DOWN;
    else if(dir == UPDIRECTION)
        keyCode = KEY.ARROW_UP;
    else if(dir == LEFTDIRECTION)
        keyCode = KEY.ARROW_LEFT;
    else if(dir == RIGHTDIRECTION)
        keyCode = KEY.ARROW_RIGHT;
    else
        keyCode = KEY.ARROW_LEFT;

    // document.dispatchEvent(new KeyboardEvent('keydown',{'keyCode':keyCode}));
    var keyboardEvent = new KeyboardEvent('keydown', {bubbles:true});
    Object.defineProperty(keyboardEvent, 'keyCode', {get:function(){return keyCode;}});
    keyboardEvent.keyCode = [keyCode];
    document.body.dispatchEvent(keyboardEvent);
}

function doSimulationStep(){
	
}