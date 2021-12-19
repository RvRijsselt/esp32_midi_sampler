#pragma once

#include <Arduino.h>

typedef void(*FuncPtrCallback)(uint8_t userdata, float value);

void Web_Setup();
void Web_AddSlider(std::string name, FuncPtrCallback callback, int maxValue=255);
void Web_AddLine();