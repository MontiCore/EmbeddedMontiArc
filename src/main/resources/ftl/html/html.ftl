<#-- (c) https://github.com/MontiCore/monticore -->
<#ftl strip_whitespace=true>
<!doctype html>
<html lang="en-us">
<style>
  .model {
    overflow: hidden;
    width: fit-content;
  }
  .ports {
    overflow: hidden;
  }
  .inports {
    float: left;
    margin: 10px;
  }
  .outports {
    float: left;
    margin: 10px;
  }

  .header {
    margin: 0 auto;
    width: fit-content;
  }

  .type {
    width: max-content;
    margin: 2px 15px 2px 2px;
    color: grey;
  }
  .buttons {
    display: flex;
    flex-direction: row;
    justify-content: space-evenly;
    margin: 10px;
  }
  .button {
    color: white;
    padding: 10px 16px;
    border: none;
    text-align: center;
    text-decoration: none;
    font-size: 14px;
    cursor: pointer;
    width: 100px;
  }
  #reset {
    background-color: #d22b23;
  }
  #execute {
    background-color: #24d231;
  }
  .error {
    color: red;
    margin: 10px;
  }

  textarea {
    height: 1em;
    width: 200px;
    padding: 3px;
  }
</style>
<head>
  <script type="text/javascript"
          src="https://cdnjs.cloudflare.com/ajax/libs/mathjs/4.0.1/math.min.js"></script>
  <script type="text/javascript" src="${model_wrapper}.js"></script>
  <script type="text/javascript" src="${model}.js"></script>
  <title>${model_name}</title>
</head>
<body>
<div class="container">
  <div class="model">
    <div class="header">
      <h2>${model_name}</h2>
    </div>
    <div class="ports">
      <div class="inports">
        <div class="header">
          <h3>Inports</h3>
        </div>
        <table>
        <#list inports as inport>
          <tr>
            <td>
              <div class="type">${inport.type}</div>
            </td>
            <td>
              <div class="label">
                ${inport.name}:
              </div>
            </td>
            <td>
              <div class="field">
                <textarea id="inport-field-${inport.name}"></textarea>
              </div>
            </td>
          </tr>
        </#list>
        </table>
      </div>
      <div class="outports">
        <div class="header">
          <h3>Outports</h3>
        </div>
        <table>
        <#list outports as outport>
          <tr>
            <td>
              <div class="type">${outport.type}</div>
            </td>
            <td>
              <div class="label">
                ${outport.name}:
              </div>
            </td>
            <td>
              <div class="field">
                <textarea id="outport-field-${outport.name}" readonly></textarea>
              </div>
            </td>
          </tr>
        </#list>
        </table>
      </div>
    </div>
    <div class="buttons">
      <div class="reset">
        <button class="button" id="reset" onclick="reset()">Reset</button>
      </div>
      <div class="execute">
        <button class="button" id="execute" onclick="exec()">Execute</button>
      </div>
    </div>
  </div>
  <div class="error" id="error"></div>
</div>

<script>
  var url_string = window.location.href;
  var url = new URL(url_string);
  <#list inports as inport>
    document.getElementById("inport-field-${inport.name}").value = url.searchParams.get("${inport.name}");
  </#list>

  function exec() {
    clearOutportFields();
    clearErrors();

    try {
    <#list inports as inport>
      ${inport.wrapperFunction}(document.getElementById("inport-field-${inport.name}").value);
    </#list>

      execute();

    <#list outports as outport>
      document.getElementById("outport-field-${outport.name}").value = ${outport.wrapperFunction}();
    </#list>
    }
    catch (err) {
      if (err.message === undefined) {
        document.getElementById("error").innerText = err;
      }
      else {
        document.getElementById("error").innerHTML = err.message;
      }
    }
  }

  function reset() {
    init();
    clearInportFields();
    clearOutportFields();
    clearErrors();
  }

  function clearInportFields() {
  <#list inports as inport>
    document.getElementById("inport-field-${inport.name}").value = "";
  </#list>
  }

  function clearOutportFields() {
  <#list outports as outport>
    document.getElementById("outport-field-${outport.name}").value = "";
  </#list>
  }

  function clearErrors() {
    document.getElementById("error").innerHTML = "";
  }
</script>
</body>
</html>
