#include "web.h"
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>

#include "wifi_credentials.h"

AsyncWebServer server(80);
String devices;

FuncPtrCallback sliderCallbacks[20];
int nrOfDevices = 0;

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Cuculin Web Server</title>
  <style>
    html {font-family: Arial; display: inline-block; text-align: center;}
    h2 {font-size: 2.3rem;}
    p {font-size: 1.9rem;}
    body {max-width: 400px; margin:0px auto; padding-bottom: 25px;}
    .slider { -webkit-appearance: none; margin: 14px; width: 360px; height: 25px; background: #FFD65C; outline: none; -webkit-transition: .2s; transition: opacity .2s;}
    .slider::-webkit-slider-thumb {-webkit-appearance: none; appearance: none; width: 35px; height: 35px; background: #003249; cursor: pointer;}
    .slider::-moz-range-thumb { width: 35px; height: 35px; background: #003249; cursor: pointer; } 
    hr { border: none; background-image: linear-gradient(to right, rgba(0, 0, 0, 0), rgb(255, 0, 200), rgba(0, 0, 0, 0)); height: 9px; }
  </style>
</head>
<body>
  <h2>Cuculin Web Server</h2>
<script>
 var tId;
 function updtRange(el) {
     clearTimeout(tId);
     func = function() {
        document.getElementById("value-"+el.id).innerHTML = el.value;
        var xhr = new XMLHttpRequest();
        xhr.open("GET", "/slider?value="+el.value+"&id="+el.dataset.index, true);
        xhr.send();
     }
     tId = setTimeout(func, 100);
 }
 function addSlider(name, id, maxValue) {
    var div = document.createElement('div');
    div.innerHTML = "<p>" + name + ": <span id='value-slider-" + id + "'></span></p>"
                  + "<p><input type='range' oninput='updtRange(this)' id='slider-" + id + "' data-index='"+ id + "' min='0' max='" + maxValue + "' value='127' step='1' class='slider'></p>";
    document.body.appendChild(div); 
 }
 function addBtn(name, id) {
    var btn = document.createElement('button');
    btn.innerHTML = name;
    btn.addEventListener("click", function() {
        var xhr = new XMLHttpRequest();
        xhr.open("GET", "/slider?value=255&id="+id, true);
        xhr.send();
    });
    document.body.appendChild(btn); 
 }
 function addHr() {
    document.body.appendChild(document.createElement('hr')); 
 }
%SCRIPT%
</script>
</body>
</html>
)rawliteral";

String processor(const String& var){
    if (var == "SCRIPT") {
        return devices;
    }
    return String();
}

void Web_Setup()
{
    Serial.print("Connecting to wifi...");

    WiFi.mode(WIFI_STA);
    WiFi.setAutoConnect(true);
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
    }
    Serial.println(" done.");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    WiFi.setSleep(false);


    devices.reserve(1000);

    // Set up mDNS responder:
    //if (MDNS.begin("cuculin")) {
    //    // Add service to MDNS-SD
    //    MDNS.addService("http", "tcp", 80);
    //    Serial.println("mDNS responder started");
    //} else {
    //    Serial.println("Error setting up MDNS responder!");
    //}

    // Route for root / web page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        Serial.printf("Web connection");
        request->send_P(200, "text/html", index_html, processor);
    });

    // Send a GET request to <ESP_IP>/slider?value=<inputMessage>&id=<index>
    server.on("/slider", HTTP_GET, [] (AsyncWebServerRequest *request) {
        Serial.printf("Slider %s", request->url().c_str());
        if (request->hasParam("value") && request->hasParam("id")) {
            auto value = request->getParam("value")->value().toInt() / 256.0f;
            auto index = request->getParam("id")->value().toInt();
            if (index >= 0 && index < nrOfDevices) {
                sliderCallbacks[index](2, value);
            }
        }
        request->send(200, "text/plain", "OK");
    });

    // Start server
    server.begin();
}

void Web_AddButton(const char *name, FuncPtrCallback callback) {
    const int new_id = nrOfDevices;
    char outstr[70];

    sprintf(outstr, "addBtn(\"%s\", %d);\n", name, new_id);
    devices += outstr;

    sliderCallbacks[new_id] = callback;
    nrOfDevices++;
}

void Web_AddSlider(const char *name, FuncPtrCallback callback, int maxValue) {
    const int new_id = nrOfDevices;
    char outstr[90];

    sprintf(outstr, "addSlider(\"%s\", %d, %d);\n", name, new_id, maxValue);
    devices += outstr;

    sliderCallbacks[new_id] = callback;
    nrOfDevices++;
}

void Web_AddLine() {
    devices += "addHr();\n";
}
