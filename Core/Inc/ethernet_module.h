#ifndef  _ETHERNET_MODULE_H_
#define  _ETHERNET_MODULE_H_

#include "main.h"

#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include "socket.h"
#include "dhcp.h"
#include "httpServer.h"

#include "ssd1306.h"
#include "ssd1306_fonts.h"

#define DHCP_SOCKET     0
#define DNS_SOCKET      1
#define MAX_HTTPSOCK    6

#define index_page "<!DOCTYPE html>"\
  "<html>"\
    "<head>"\
      "<title>W5500-STM32 Web Server</title>"\
      "<meta http-equiv='Content-Type' content='text/html; charset=utf-8'>"\
      "<link href=\"data:image/x-icon;base64,A\" rel=\"icon\" type=\"image/x-icon\">"\
      "<style>"\
        "html {display: inline-block; margin: 0px auto; text-align: center;}"\
        "body{margin-top: 50px;}"\
        ".button {display: block;"\
          "width: 70px;"\
          "background-color: #008000;"\
          "border: none;"\
          "color: white;"\
          "padding: 14px 28px;"\
          "text-decoration: none;"\
          "font-size: 24px;"\
          "margin: 0px auto 36px;"\
          "border-radius: 5px;}"\
        ".button-on {background-color: #008000;}"\
        ".button-on:active{background-color: #008000;}"\
        ".button-off {background-color: #808080;}"\
        ".button-off:active {background-color: #808080;}"\
        "p {font-size: 20px;color: #808080;margin-bottom: 20px;}"\
      "</style>"\
    "</head>"\
    "<body>"\
      "<h1>STM32 - W5500</h1>"\
      "<p>Control the light via Ethernet</p>"\
      "<a class=\"button button-on\" href=\"/ledon.html\">ON</a>"\
      "<a class=\"button button-off\" href=\"/ledoff.html\">OFF</a>"\
    "</body>"\
  "</html>"


void W5500Init();
void handleHtpServer();

#endif // _ETHERNET_MODULE_H_
