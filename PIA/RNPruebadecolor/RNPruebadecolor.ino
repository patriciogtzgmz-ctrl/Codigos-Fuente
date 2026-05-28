#include "esp_camera.h"
#include <math.h>

// ===============================
// CONFIGURACION DE LA ESP32-CAM
// ===============================
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


// ===============================
//  PESOS DE LA RED  3 → 16 → 8 → 1
// ===============================

// Capa 1  (3 entradas → 16 neuronas)
float W1[3][16] = {
  { 0.7034929,  -0.04974367,  0.38486695,  0.09586847, -0.43130964, -0.2531054,
   -0.5603879,   0.2750517,   0.13045746, -0.38361576,  0.18424048, -0.53622645,
    0.32033294, -0.97392887, -0.3168381,  -0.8448642  },
  {-0.25413904, -0.48027068,  0.54092616, -0.4638198,  -0.08594355, -0.766478,
   -0.09303775, -0.28232947, -0.27285776, -0.06002683,  0.5604879,  -0.84192675,
    0.41571087,  0.27856132, -0.2893674,   0.5391882  },
  { 0.18119518,  0.71888334, -0.26458922, -0.46642852, -0.38725096,  0.45796207,
   -0.16202924, -0.48461264, -0.20884898,  0.2676802,   0.46943575,  0.5720737,
   -0.6657567,   0.3731887,  -0.13471884,  0.0914224  }
};

float b1[16] = {
   0.02868199,  0.08817562, -0.02013456,  0.0,         0.0,         0.09022576,
   0.0,        -0.05194514,  0.0,        -0.17369251,  0.18681882,  0.11840374,
  -0.0427959,   0.16676669,  0.0,         0.0364908
};

// Capa 2  (16 → 8)
float W2[16][8] = {
  { 0.09919801, -1.1056535,  -0.11656386, -0.4980653,  -0.10458481, -0.13208552, -0.25131577,  0.5402913  },
  {-0.02711485, -0.08740739, -0.7446997,   0.3944235,  -0.43098915,  0.20620632,  0.44818497,  0.2099125  },
  {-0.0723673,  -0.01705264,  0.34204555, -0.00426203, -0.18486488,  0.02401642, -0.20637818, -0.04559412},
  {-0.21265745,  0.1745956,  -0.06199992,  0.44742393,  0.24981248, -0.13319099,  0.20477998,  0.44927025},
  { 0.32999563, -0.498448,    0.17542195, -0.22592998,  0.05317366, -0.39624894, -0.08713365, -0.410406  },
  {-0.2211437,   0.72615546, -0.5414794,  -0.0321122,   0.28469718, -0.34185457,  0.47609824, -0.401749  },
  {-0.372939,   -0.05225456, -0.20887077,  0.31166887,  0.06829393,  0.16728997, -0.24805593, -0.46646237},
  {-0.3463982,  -0.20725536, -0.14072804, -0.31772602, -0.09880686,  0.10655188,  0.00261533,  0.30109355},
  { 0.44306386, -0.20751405,  0.19958961, -0.31602216,  0.46632326, -0.4737879,   0.01043046,  0.04696536},
  { 0.06846796, -0.2847685,   0.40437216,  0.41726133,  0.48205292, -0.28994167, -0.28824058,  0.10899657},
  { 0.15934215,  0.29336563,  0.3618551,  -0.40722668, -0.12319219, -0.35913736,  0.00797564,  0.45255688},
  { 0.38267893,  1.3604805,   0.15389547,  0.06341271, -0.39904618,  0.34661615, -0.956138,   -0.8826044 },
  { 0.57644767, -0.15597588,  0.63495153,  0.08779562, -0.3672042,   0.32238978,  0.08044238, -0.36426967},
  {-0.3021906,   0.8644146,   0.00967173, -0.39223287, -0.25856352, -0.06721365, -0.26325813, -0.83791745},
  { 0.13925588, -0.46595287, -0.24427438, -0.3778273,  -0.42967367, -0.33414638,  0.0881325,   0.18761253},
  {-0.5260191,   0.7184714,   0.9280134,   0.2834082,  -0.08049572, -0.08102667, -0.5683926,  -0.49978504}
};

float b2[8] = {
  -0.17592081,  0.15565845, -0.0244599, -0.04717037,
   0.0,        -0.019571,    0.08526979,  0.13871928
};

// Capa 3  (8 → 1)
float W3[8] = {
  -0.3982852, -1.4426317, -0.80838627,  0.56871724,
  -0.77453965, 0.6533581,  0.9426462,   1.273187
};

float b3 = -0.02245039;


// ===============================
//  NORMALIZACIÓN HSV
//  H: 0–360 → 0–1   S,V: ya están en 0–1
// ===============================
void normalizarHSV(float H, float S, float V, float* out) {
  out[0] = H / 360.0;
  out[1] = S;
  out[2] = V;
}


// ===============================
//  CONVERSIÓN RGB → HSV
//  R,G,B en 0–255
// ===============================
void rgbToHSV(int R, int G, int B, float* H, float* S, float* V) {
  float r = R / 255.0;
  float g = G / 255.0;
  float b = B / 255.0;

  float cmax = max(r, max(g, b));
  float cmin = min(r, min(g, b));
  float diff = cmax - cmin;

  *V = cmax;
  *S = (cmax == 0) ? 0 : diff / cmax;

  if (diff == 0) {
    *H = 0;
  } else if (cmax == r) {
    *H = fmod(60.0 * ((g - b) / diff) + 360.0, 360.0);
  } else if (cmax == g) {
    *H = fmod(60.0 * ((b - r) / diff) + 120.0, 360.0);
  } else {
    *H = fmod(60.0 * ((r - g) / diff) + 240.0, 360.0);
  }
}


// ===============================
//  ACTIVACIONES
// ===============================
float relu(float x) {
  return x > 0 ? x : 0;
}

float sigmoid(float x) {
  return 1.0 / (1.0 + exp(-x));
}


// ===============================
//  INFERENCIA  (entrada ya normalizada)
// ===============================
int predecir(float* x) {

  // Capa 1: 3 → 16
  float h1[16];
  for (int j = 0; j < 16; j++) {
    float s = b1[j];
    for (int i = 0; i < 3; i++)
      s += x[i] * W1[i][j];
    h1[j] = relu(s);
  }

  // Capa 2: 16 → 8
  float h2[8];
  for (int j = 0; j < 8; j++) {
    float s = b2[j];
    for (int i = 0; i < 16; i++)
      s += h1[i] * W2[i][j];
    h2[j] = relu(s);
  }

  // Capa 3: 8 → 1
  float out = b3;
  for (int j = 0; j < 8; j++)
    out += h2[j] * W3[j];

  float prob = sigmoid(out);

  Serial.print("Probabilidad amarillo: ");
  Serial.println(prob, 4);

  return prob > 0.5 ? 1 : 0;
}


// ===============================
//  SETUP
// ===============================
void setup() {
  Serial.begin(115200);

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_RGB565;
  config.frame_size   = FRAMESIZE_QQVGA;
  config.fb_count     = 1;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("ERROR: No se pudo iniciar la camara");
    return;
  }

  Serial.println("Camara lista!");
}


// ===============================
//  LOOP
// ===============================
void loop() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Error capturando imagen");
    return;
  }

  // Pixel del centro
  int cx = fb->width  / 2;
  int cy = fb->height / 2;

  uint16_t pixel = ((uint16_t*)fb->buf)[cy * fb->width + cx];

  // Decodificar RGB565
  int R = ((pixel >> 11) & 0x1F) << 3;
  int G = ((pixel >> 5)  & 0x3F) << 2;
  int B = ( pixel        & 0x1F) << 3;

  Serial.print("RGB: ");
  Serial.print(R); Serial.print(", ");
  Serial.print(G); Serial.print(", ");
  Serial.println(B);

  // RGB → HSV
  float H, S, V;
  rgbToHSV(R, G, B, &H, &S, &V);

  Serial.print("HSV: ");
  Serial.print(H, 2); Serial.print(", ");
  Serial.print(S, 2); Serial.print(", ");
  Serial.println(V, 2);

  // Normalizar y predecir
  float entrada[3];
  normalizarHSV(H, S, V, entrada);

  int resultado = predecir(entrada);

  if (resultado == 1)
    Serial.println(">> COLOR AMARILLO DETECTADO");
  else
    Serial.println(">> NO ES AMARILLO");

  Serial.println("----------------------------");

  esp_camera_fb_return(fb);
  delay(500);
}