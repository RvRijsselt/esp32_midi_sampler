#pragma once

#include <Arduino.h>

typedef void(*FuncPtrCallback)(uint8_t userdata, float value);

void Web_Setup();
void Web_ProcessLoop();
void Web_AddButton(const char *name, FuncPtrCallback callback);
void Web_AddSlider(const char *name, FuncPtrCallback callback, int maxValue=255);
void Web_AddRecSelector();
void Web_AddLine();