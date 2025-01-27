#include "ethernet_module.h"

extern SPI_HandleTypeDef hspi1;
extern int16_t encoderValue;

static uint8_t dhcp_buffer[1024]; // the 'static' is needed when the optimization is enable, why? idk ._.

static uint8_t RX_BUF[1024];
static uint8_t TX_BUF[1024];

uint8_t socknumlist[] = {2, 3, 4, 5, 6, 7};

wiz_NetInfo net_info = {
    .mac  = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED },
    .dhcp = NETINFO_DHCP
};

bool ip_assigned = false;

/*                HTTP CALLBACKS                 */
void led_on(uint8_t **content, uint32_t *content_len) {
  uint8_t *mem = malloc(10);
  strcpy((char*)mem, "LED ON");

  *content = mem;
  *content_len = strlen((char*)mem);

  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

void led_off(uint8_t **content, uint32_t *content_len) {
  uint8_t *mem = malloc(10);
  strcpy((char*)mem, "LED OFF");

  *content = mem;
  *content_len = strlen((char*)mem);
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
}

void get_encoder_value(uint8_t **content, uint32_t *content_len) {
  char str[10];
  sprintf(str, "%d", encoderValue);

  uint8_t *mem = malloc(10);
  strcpy((char*)mem, str);
  *content = mem;
  *content_len = strlen((char*)mem);
}
/*                HTTP CALLBACKS                 */


void wizchipSelect(void) {
    HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_RESET);
}

void wizchipUnselect(void) {
    HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_SET);
}

void wizchipReadBurst(uint8_t* buff, uint16_t len) {
    HAL_SPI_Receive(&hspi1, buff, len, HAL_MAX_DELAY);
}

void wizchipWriteBurst(uint8_t* buff, uint16_t len) {
    HAL_SPI_Transmit(&hspi1, buff, len, HAL_MAX_DELAY);
}

uint8_t wizchipReadByte(void) {
    uint8_t byte;
    wizchipReadBurst(&byte, sizeof(byte));
    return byte;
}

void wizchipWriteByte(uint8_t byte) {
    wizchipWriteBurst(&byte, sizeof(byte));
}


void Callback_IPAssigned(void) {
    ip_assigned = true;
}

void Callback_IPConflict(void) {
    ip_assigned = false;
}

void W5500Init() {

    // Register W5500 callbacks
    reg_wizchip_cs_cbfunc(wizchipSelect, wizchipUnselect);
    reg_wizchip_spi_cbfunc(wizchipReadByte, wizchipWriteByte);
    reg_wizchip_spiburst_cbfunc(wizchipReadBurst, wizchipWriteBurst);

    uint8_t rx_tx_buff_sizes[] = {2, 2, 2, 2, 2, 2, 2, 2};
    wizchip_init(rx_tx_buff_sizes, rx_tx_buff_sizes);

    // set MAC address before using DHCP
    setSHAR(net_info.mac);
    DHCP_init(DHCP_SOCKET, dhcp_buffer);

    reg_dhcp_cbfunc(
        Callback_IPAssigned,
        Callback_IPAssigned,
        Callback_IPConflict
    );

    ssd1306_SetCursor(0, 12);
    ssd1306_WriteString("Waiting IP", Font_7x10, White);
    ssd1306_UpdateScreen();

    uint32_t ctr = 10000;
    while((!ip_assigned) && (ctr > 0)) {
      ssd1306_SetCursor(72, 12);
      if (ctr % 2 == 0) {
        ssd1306_WriteString("   ", Font_7x10, White);
      } else {
        ssd1306_WriteString("...", Font_7x10, White);
      }
      ssd1306_UpdateScreen();

      printf("Waiting IP...\n");
      DHCP_run();
      HAL_Delay(500);
      ctr--;
    }

    if(!ip_assigned) {
      ssd1306_SetCursor(0, 12);
      ssd1306_WriteString("IP not assigned", Font_7x10, White);
      ssd1306_UpdateScreen();
      return;
    }

    getIPfromDHCP(net_info.ip);
    getGWfromDHCP(net_info.gw);
    getSNfromDHCP(net_info.sn);

    char ipStr[30];
    sprintf(ipStr, "IP: %d.%d.%d.%d", net_info.ip[0], net_info.ip[1], net_info.ip[2], net_info.ip[3]);
    ssd1306_SetCursor(0, 12);
    ssd1306_WriteString(ipStr, Font_7x10, White);
    ssd1306_UpdateScreen();

    wizchip_setnetinfo(&net_info);

    // set the TCP Timeout, needed when the connection is lost/cut in the middle of a request
    wiz_NetTimeout timeout;
    wizchip_gettimeout(&timeout);
    printf("OLD TIMEOUT: count: %d, time_100us: %d \n", timeout.retry_cnt, timeout.time_100us);

    timeout.retry_cnt = 4;
    timeout.time_100us = 1000;
    wizchip_settimeout(&timeout);

    wizchip_gettimeout(&timeout);
    printf("NEW TIMEOUT: count: %d, time_100us: %d \n", timeout.retry_cnt, timeout.time_100us);


    // setup http server
    httpServer_init(TX_BUF, RX_BUF, MAX_HTTPSOCK, socknumlist);

    /* Web content registration */
    reg_httpServer_webContent((uint8_t *)"index.html", (uint8_t *)index_page);

    /* Register function to respond to API call */
    reg_httpServer_api((uint8_t *)"api/led/on", led_on);
    reg_httpServer_api((uint8_t *)"api/led/off", led_off);
    reg_httpServer_api((uint8_t *)"api/encoder", get_encoder_value);
}

void handleHtpServer() {
  for(int i = 0; i < MAX_HTTPSOCK; i++) httpServer_run(i); // HTTP Server handler
}


