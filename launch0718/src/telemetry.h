#pragma once

void setupTelemetry();
void printBoth(const char *text);
void printfBoth(const char *format, ...);
void handleDtuTelemetry();
void sendDtuStatusNow();
