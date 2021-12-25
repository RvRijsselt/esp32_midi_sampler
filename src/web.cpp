#include "web.h"
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>
#include <DNSServer.h>

#include "wifi_credentials.h"

//#define USE_AP

#ifdef USE_AP
DNSServer dnsServer;
#endif
AsyncWebServer server(80);
String devices;

#define MAX_DEVICES 30
FuncPtrCallback sliderCallbacks[MAX_DEVICES];
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

class CaptiveRequestHandler : public AsyncWebHandler {
public:
  CaptiveRequestHandler() {}
  virtual ~CaptiveRequestHandler() {}

  bool canHandle(AsyncWebServerRequest *request){
    return true;
  }

  void handleRequest(AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", index_html); 
  }
};

void Web_Setup()
{
    devices.reserve(1000);

    Serial.print("Connecting to wifi...");

#ifndef USE_AP
    WiFi.mode(WIFI_STA);
    WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
    WiFi.setHostname("cuculin");
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
#else
    IPAddress localIP(192,168,1,1);
    IPAddress subnet(255,255,255,0);
    WiFi.mode(WIFI_AP);
    WiFi.setHostname("cuculin");
    WiFi.softAP("cuculin", "kukukuku");
    delay(200);
    WiFi.softAPConfig(localIP, localIP, subnet);
    delay(200);
    Serial.print("AP IP address: ");Serial.println(WiFi.softAPIP());
#endif

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
        static int cnt = 0;
        int req = ++cnt;
        Serial.printf("Web connection %d\n", req);
        Serial.printf("ESP.getFreeHeap() %d\n", ESP.getFreeHeap());
        Serial.printf("ESP.getMinFreeHeap() %d\n", ESP.getMinFreeHeap());
        Serial.printf("ESP.getHeapSize() %d\n", ESP.getHeapSize());
        Serial.printf("ESP.getMaxAllocHeap() %d\n", ESP.getMaxAllocHeap());
        request->send_P(200, "text/html", index_html, processor);
        Serial.printf("Web connection %d done\n", req);
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

#ifdef USE_AP
    dnsServer.start(53, "*", WiFi.softAPIP());
    server.addHandler(new CaptiveRequestHandler()).setFilter(ON_AP_FILTER);//only when requested from AP
#endif

    // Start server
    server.begin();
}

void Web_ProcessLoop() {
#ifdef USE_AP
    dnsServer.processNextRequest();
#endif
}

void Web_AddButton(const char *name, FuncPtrCallback callback) {
    if (nrOfDevices >= MAX_DEVICES) { Serial.println("Too many devices"); while(1){}}

    const int new_id = nrOfDevices;
    char outstr[70];

    sprintf(outstr, "addBtn(\"%s\", %d);\n", name, new_id);
    devices += outstr;

    sliderCallbacks[new_id] = callback;
    nrOfDevices++;
}

void Web_AddSlider(const char *name, FuncPtrCallback callback, int maxValue) {
    if (nrOfDevices >= MAX_DEVICES) { Serial.println("Too many devices"); while(1){}}

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
