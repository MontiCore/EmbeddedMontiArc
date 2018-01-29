/**
 * I set up the listeners for dragging and dropping as well
 * as creating an iFrame for holding dragged in images
 * @returns {undefined}
 */
function main() {
    // The div that receives drops and the new iFrame
    var targetDiv = document.getElementById("dropTarget"),
        iframe = document.createElement("iframe");

    // Set the iframe to a blank page
    iframe.src = "about:blank";

    // Append it to the target
    document.getElementById("iframeTarget").appendChild(iframe);
    document.getElementById("iframeTarget").width="1000";
    // Drag over is when an object is hovering over the div
    // e.preventDefault keeps the page from changing
    targetDiv.addEventListener("dragover", function (e) {
        e.preventDefault();
        this.className = "dropMe";
    }, false);

    // Drag leave is when the object leaves the div but isn't dropped
    targetDiv.addEventListener("dragleave", function (e) {
        e.preventDefault();
        this.className = "dropEnabled";
    }, false);

    // Drop is when the click is released
    targetDiv.addEventListener("drop", function (e) {
        e.preventDefault();
        this.className = "dropEnabled";
        loadFile(e.dataTransfer.files[0], iframe);
    }, false);

    document.getElementById("upload").addEventListener("click", function () {
        var file = document.getElementById("browsedFile").files[0];
        loadFile(file, iframe);
    }, false);
}

/**
 * Load a file and then put it on an ifrmae
 * @param {Element} f The file that needs to get loaded
 * @param {Element} destination The iframe that the file is appended to
 * @returns {undefined}
 */
function loadFile(f, destination) {
    // Make a file reader to interpret the file
    var reader = new FileReader();

    // When the reader is done reading,
    // Make a new image tag and append it to the iFrame
    reader.onload = function (event) {
        var newImage = document.createElement("img");
        newImage.src = event.target.result;
        destination.contentWindow.document.getElementsByTagName("body")[0].appendChild(newImage);
        if(destination.contentWindow.document.getElementsByTagName("body")[0].childElementCount>1){
            destination.contentWindow.document.getElementsByTagName("body")[0].removeChild(destination.contentWindow.document.getElementsByTagName("body")[0].firstChild);
        }
        $('#iframeTarget').height(1000);
        $('#iframeTarget').width(1000);
        console.log("Loaded File");
        
    };

    // Tell the reader to start reading asynchrounously
    reader.readAsDataURL(f);
}

// Run the main script
//window.onload = main;