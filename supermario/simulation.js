var KEY = {
	UP: 38,
	DOWN: 40,
	LEFT: 37,
	RIGHT: 39,
	SHOOT: 17,
}



function startSimulation() {
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
    var keyCode = dir;


    // document.dispatchEvent(new KeyboardEvent('keydown',{'keyCode':keyCode}));
    var keyboardEvent = new KeyboardEvent('keydown', {bubbles:true});
    Object.defineProperty(keyboardEvent, 'keyCode', {get:function(){return keyCode;}});
    keyboardEvent.keyCode = [keyCode];
    document.body.dispatchEvent(keyboardEvent);
}

function sendPlayerData(){

	var player = FSM.player;
	var marioPos = [player.left, player.top];
	var marioVel = [player.xvel, player.yvel];
	var marioHeight = player.height;
	
	setMarioPosition(marioPos);
	setMarioVelocity(marioVel);
	setMarioHeight(marioHeight);
	
	console.log(player);
}

function sendEnvironmentData(){
	var solid = FSM.GroupHolder.group.solid;
	
	
	var nextObstacles = extractNextObstaclePositions(solid);
	var nextEnemies = extractNextObstaclePositions(solid);
	var nextHole = extractNextHolePosition(solid);
	
	
	setNextObstaclePositions(nextObstaclePositions);
	setNextEnemyPositions(nextEnemyPositions);
	setNextHole(nextHole);
	

}

function getCommands(){
	var dir = getMarioDirection();
	var jump = getMarioJump();
	var shoot = getMarioShoot();
	
	if(dir==1)
		pressKey(KEY.RIGHT);
	else
		if(dir==-1)
			pressKey(KEY.RIGHT);
		
	if(jump==true)
		pressKey(KEY.UP);
	
	if(shoot==true)
		pressKey(KEY.CONTROL);
	
	
	
}

function doSimulationStep(){
	sendPlayerData();
	sendEnvironmentData();
	
	getCommands();
}


