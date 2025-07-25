# PacMan
An EMA component to control PacMan in the game **PacMan**.  
To insert your own controller, exchange the component *controller* within *PacManWrapper* with your controller.

## Features
* **Execute Model**: use the compiled controller to simulate a game of **PacMan**
* **Generate WebAssembly**: compile the controller to WebAssembly
* **Visualize Workspace**: generate a visualization of the model
* **Show Report**: generate a report for all the components in this workspace
* **Show Report with Streams**: generate a report for all the components in this workspace including stream tests results
* **Test Workspace**: run all stream tests and show the results
* **Test active Stream**: run the custom stream test and show the result
* **Debug Model**: compile the model, generate a visualization and show a debugging tool
* **Debug Model (Without SVG)**: compile the model and show a debugging tool
* **Play PacMan**: play a game of **PacMan**

###Note: 
**Debug Model** might take some time to execute because of the visualization generation. This is only neccessary if a component is changed outside its *implementation Math*-block. Otherwise **Debug Model (Without SVG)** can be used.

When resizing the windows inside the debugging tool with the black bar, you might need to scroll within the blue area.
