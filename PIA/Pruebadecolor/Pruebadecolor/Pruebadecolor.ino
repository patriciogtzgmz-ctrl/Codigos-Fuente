#include "esp_camera.h"

// =============== CONFIGURACION ESP32-CAM ===============
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22


// ================= RGB → HSV =================
void rgb2hsv(uint8_t r, uint8_t g, uint8_t b, float &h, float &s, float &v) {
    float rf = r / 255.0;
    float gf = g / 255.0;
    float bf = b / 255.0;

    float cmax = max(rf, max(gf, bf));
    float cmin = min(rf, min(gf, bf));
    float diff = cmax - cmin;

    // Hue
    if (diff == 0) h = 0;
    else if (cmax == rf) h = 60 * fmod(((gf - bf) / diff), 6);
    else if (cmax == gf) h = 60 * (((bf - rf) / diff) + 2);
    else h = 60 * (((rf - gf) / diff) + 4);

    if (h < 0) h += 360;

    // Saturation
    s = (cmax == 0 ? 0 : diff / cmax);

    // Value
    v = cmax;
}



// =============== DETECCION DE COLOR ===============

// ------ Amarillo original ------
bool esAmarilloTipo1(float H, float S, float V) {

    bool rangoH = (H >= 120 && H <= 300);   // Rango amplio de 150 a 300
    bool rangoS = (S >= 0.50 && S <= 0.90); // Tu rango pedido
    bool rangoV = (V >= 0.40 && V <= 0.99); // Tu rango pedido

    if (rangoH && rangoS && rangoV) {
        return true;
    }
    return false;
}


// ------ Amarillo tipo 2 (según tus datos nuevos) ------
// Promedio aproximado de tus valores:
// H ≈ 230–270
// S ≈ 0.40–0.60
// V ≈ 0.30–0.50
bool esAmarilloTipo2(float H, float S, float V) {

    bool rangoH = (H >= 200 && H <= 320);   // Rango amplio de 150 a 300
    bool rangoS = (S >= 0.10 && S <= 0.95); // Tu rango pedido
    bool rangoV = (V >= 0.50 && V <= 0.99); // Tu rango pedido

    if (rangoH && rangoS && rangoV) {
        return true;
    }
    return false;
}



// =============== SETUP ===============
void setup() {
    Serial.begin(115200);

    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_RGB565;

    config.frame_size = FRAMESIZE_QQVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;

    if (esp_camera_init(&config) != ESP_OK) {
        Serial.println("Error iniciando la camara");
        return;
    }

    Serial.println("Camara lista");
}



// =============== LOOP ===============
void loop() {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("Error capturando imagen");
        return;
    }

    // Tomamos el pixel central de referencia
    int cx = fb->width / 2;
    int cy = fb->height / 2;

    uint16_t pixel = ((uint16_t *)fb->buf)[cy * fb->width + cx];

    uint8_t r = ((pixel >> 11) & 0x1F) << 3;
    uint8_t g = ((pixel >> 5) & 0x3F) << 2;
    uint8_t b = (pixel & 0x1F) << 3;

    float H, S, V;
    rgb2hsv(r, g, b, H, S, V);

    Serial.print("HSV: ");
    Serial.print(H); Serial.print("  ");
    Serial.print(S); Serial.print("  ");
    Serial.println(V);

    if (esAmarilloTipo1(H, S, V)) {
        Serial.println("AMARILLO TIPO 1");
    }
    else if (esAmarilloTipo2(H, S, V)) {
        Serial.println("AMARILLO TIPO 2 (CALIBRADO)");
    }
    else {
        Serial.println("No es amarillo");
    }

    esp_camera_fb_return(fb);
    delay(300);
}