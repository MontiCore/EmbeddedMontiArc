var time = Date.now();

document.onreadystatechange = function (event) {
    if (event.target.readyState !== "complete") {
        return;
    }

    var UserWrapper = new UserWrappr.UserWrappr(FullScreenMario.FullScreenMario.prototype.proliferate(
        {
            "GameStartrConstructor": FullScreenMario.FullScreenMario
        }, FullScreenMario.FullScreenMario.settings.ui, true));

    console.log("It took " + (Date.now() - time) + " milliseconds to start.");
    
    UserWrapper.GameStarter.UsageHelper.displayHelpMenu();
	try{
		startSimulation();
	} catch (err) {
        if (err.message === undefined) {
            console.log(err)
        }
        else {
            console.log(err.message);
        }
    }
};
