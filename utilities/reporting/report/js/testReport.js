/* (c) https://github.com/MontiCore/monticore */
function loadJSON(file,callback) {   
  var xobj = new XMLHttpRequest();
  xobj.overrideMimeType('application/json');
  xobj.open('GET', file, true); 
  xobj.onreadystatechange = function () {
    if (xobj.readyState == 4 && xobj.status == '200') {
    // Required use of an anonymous callback as .open will NOT return a value but simply returns undefined in asynchronous mode
      callback(xobj.responseText);
     }
  };
  xobj.send(null);  
}

function createTable(data) {
  data = JSON.parse(data);
  table = $('#my-table').DataTable( {
      "fixedHeader": true,
      "data" : data,
      "order": [[ 5, "desc" ]],
      "ordering": true,
      "paging" : false,
      "columns": [
          {
              "data": "Path", 
              sort: "string" 
          },
          {
              "data": "NameEndsWithTest",
              sort:"string", type:"alt-string"
          }
      ],
      "order": [[1, 'asc']]
  } );      
};
