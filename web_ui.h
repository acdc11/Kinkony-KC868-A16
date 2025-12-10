#pragma once
#include <WebServer.h>

extern WebServer server;

// Typ handlerów
using WebHandlerFn = void (*)();

// Rejestratory handlerów - wywołaj je z setup() w szkicu przed setupWebServer()
void web_set_root_handler(WebHandlerFn h);
void web_set_status_handler(WebHandlerFn h);
void web_set_config_get_handler(WebHandlerFn h);
void web_set_config_post_handler(WebHandlerFn h);
void web_set_inverter_page_handler(WebHandlerFn h);
void web_set_inverter_status_handler(WebHandlerFn h);
void web_set_inverter_cmd_handler(WebHandlerFn h);
void web_set_io_page_handler(WebHandlerFn h);
void web_set_io_state_handler(WebHandlerFn h);
void web_set_io_set_handler(WebHandlerFn h);
void web_set_control_handler(WebHandlerFn h);

// Inicjalizacja serwera WWW (rejestruje endpointy i uruchamia server.begin())
void setupWebServer();