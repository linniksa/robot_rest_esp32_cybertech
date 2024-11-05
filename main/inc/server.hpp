#pragma once
#include "esp_http_server.h"
#include "esp_wifi_types_generic.h"
#include <stdio.h>

class Server {
private:
  uint8_t retry_num = 0;
  static char chip_id[17]; // 16 byte + '/0'
  char ip_addr[20] = {};

  void wifi_init_sta(void);
  void start_rest_server(void);
  void event_handler(esp_event_base_t event_base, int32_t event_id,
                     void *event_data);
  static void event_handler_stat(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data) {
    static_cast<Server *>(arg)->event_handler(event_base, event_id, event_data);
  }

  static esp_err_t sensor_post_handler(httpd_req_t *req);
  static esp_err_t motor_put_handler(httpd_req_t *req);
  static esp_err_t move_put_handler(httpd_req_t *req);

  const httpd_uri_t motor = {.uri = "/motor",
                             .method = HTTP_PUT,
                             .handler = motor_put_handler,
                             .user_ctx = NULL};

  const httpd_uri_t move = {.uri = "/move",
                            .method = HTTP_PUT,
                            .handler = move_put_handler,
                            .user_ctx = NULL};

  const httpd_uri_t sensor = {.uri = "/sensor",
                              .method = HTTP_POST,
                              .handler = sensor_post_handler,
                              .user_ctx = NULL};

public:
  void init();
  Server(/* args */);
  ~Server();
};