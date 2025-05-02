#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <arch/cxd56xx/scu.h>
#include <arch/cxd56xx/adc.h>
#include <Camera.h>
#include <Stepper.h>
#include <Adafruit_ILI9341.h>
#include <SDHCI.h>

// Focus A+ (BLACK)
// Focus A- (BLUE) 
// Focus B+ (RED)
// Focus B- (WHITE)

// Zoom A+ (ORANGE)
// Zoom A- (GREEN)
// Zoom B+ (YELLOW)
// Zoom B- (PURPLE)

#define ZOOM_AP (21)
#define ZOOM_AN (20)
#define ZOOM_BP (19)
#define ZOOM_BN (18)


#define FOCUS_AP (17)
#define FOCUS_AN (16)
#define FOCUS_BP (15)
#define FOCUS_BN (14)

#define MOTOR_EN (24)

#define TFT_DC  9
#define TFT_CS  10

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);
SDClass theSD;

const int turnSteps = 1607;  // 0.224 deg/step 1607 = 360/0.224
const int zoomMaxStep = 2342;
const int focusMaxStep = 2343;
const int rpm = 10;

const static int IDLE_CONTRAST_AF = 0;
const static int FAST_CONTRAST_AF = 1;
const static int FINE_CONTRAST_AF = 2;
static int mode = IDLE_CONTRAST_AF;

Stepper FocusStepper(turnSteps, FOCUS_AP, FOCUS_AN, FOCUS_BP, FOCUS_BN);
Stepper ZoomStepper(turnSteps, ZOOM_AP, ZOOM_AN, ZOOM_BP, ZOOM_BN);

// parameter for volume operation
static uint32_t last_time_ms = 0;
static uint32_t last_zoom_value = 0;

const int intPin = 2;
bool bButtonPressed = false;
void changeState() {
  bButtonPressed = true;
}


void noise_reduction(uint8_t* pImg, int width, int height) {
  const float kernel[3][3] = {{0.1, 0.1, 0.1} 
                             ,{0.1, 0.2, 0.2}
                             ,{0.1, 0.1, 0.1}};

  for (int y = 1; y < height-1; ++y) {
    for (int x = 1; x < width-1; ++x) {
      float sum = 0;
      for (int ky = -1; ky <= 1; ++ky) {
        for (int kx = -1; kx <=1; ++kx) {
          int pixel = pImg[(y + ky)*width + (x + kx)];
          float weight = kernel[ky + 1][kx + 1];
          sum += pixel * weight;          
        }
      }
      pImg[y*width + x] = (uint8_t)sum;
    }
  }
}

int contrast_value(uint8_t* pImg, int width, int height) {
  const float kernel[3][3] = {{ 1,  1,  1} 
                             ,{ 1, -8,  1}
                             ,{ 1,  1,  1}}; // laplacian
  float all_sum = 0;
  for (int y = 1; y < height-1; ++y) {
    for (int x = 1; x < width-1; ++x) {
      float sum = 0;
      for (int ky = -1; ky <= 1; ++ky) {
        for (int kx = -1; kx <=1; ++kx) {
          int pixel = pImg[(y + ky)*width + (x + kx)];
          float weight = kernel[ky + 1][kx + 1];
          sum += pixel * weight;          
        }
      }
      all_sum += abs(sum);
    }
  }
  return (int)all_sum;
}

int second_order_approximation(int index, int pre_peak, int peak, int post_peak, int step) {
  float delta = float(pre_peak - post_peak)/(2.* float((pre_peak + post_peak) - 2*peak));
  float approx_peak = float(index + delta)*step;
  return round(approx_peak); 
}


void CamCB(CamImage img) {

  static int zoom_pos  = 0;
  static int focus_pos = 0;

  static int contrast0 = 0;
  static int contrast1 = 0;
  static int contrast2 = 0;

  if (!img.isAvailable()) {
    Serial.println("img is not available");
    return;
  }

  tft.drawRGBBitmap(0, 0, (uint16_t*)img.getImgBuff(), CAM_IMGSIZE_QQVGA_H, CAM_IMGSIZE_QQVGA_V);
  img.convertPixFormat(CAM_IMAGE_PIX_FMT_GRAY);

  // ノイズリダクション
  noise_reduction(img.getImgBuff(), img.getWidth(), img.getHeight());

  // コントラストを算出
  int contrast = contrast_value(img.getImgBuff(), img.getWidth(), img.getHeight());
  contrast2 = contrast1;
  contrast1 = contrast0;
  contrast0 = contrast;

  if (mode == IDLE_CONTRAST_AF) { 

    uint16_t zoom_value = analogRead(A0);
    zoom_value = map(zoom_value, 0, 1023, 0, zoomMaxStep);

    // 直前の値との差が小さい場合は操作中と判断しスキップ
    const uint16_t scattering = 10;
    if (abs(int(zoom_value - last_zoom_value)) > scattering) {
      last_zoom_value = zoom_value;
      return;
    }
    

    const int dead_zone = 100;
    int zoom_step = zoom_value - zoom_pos;
    if (abs(zoom_step) > dead_zone) {

      const int rpm = 50;

      Serial.println("zoom_value: " + String(zoom_value));

      // ズームフォーカスレンズ制御開始
      digitalWrite(MOTOR_EN, HIGH); ...

      ZoomStepper.setSpeed(rpm);
      ZoomStepper.step(zoom_step);
      zoom_pos = zoom_value;
      mode = FAST_CONTRAST_AF;
    }

  } else if (mode == FAST_CONTRAST_AF) {

    const int fast_rpm = 50;
    const int fast_step = 300;
    const int fast_start_step = 0;
    const int fast_end_step = focusMaxStep;

    static bool fast_init = true;
    static int fast_index;
    static int fast_max_contrast;   
 

    if (fast_init) {

      Serial.println("fast_init!");
      // フォーカスモータをスタートポジションに移動
      FocusStepper.setSpeed(fast_rpm);
      FocusStepper.step(-focusMaxStep);
      fast_init = false; // 初期済み
      fast_index = 0;
      focus_pos = 0;
      fast_max_contrast = 0;

    } else {

      Serial.println("FAST -> idx: " + String(fast_index) + ", 2:" + String(contrast2)+ ", 1:" + String(contrast1) + ", 0:" + String(contrast0) + ", max:" + String(fast_max_contrast));

      if (focus_pos <= fast_end_step  && focus_pos >= fast_start_step) {

        if (contrast > fast_max_contrast) {
          fast_max_contrast = contrast;

          // まだ頂上を越えていないので、フォーカスモータをfocus_step単位で進める
          FocusStepper.step(fast_step);
          focus_pos += fast_step;
          ++fast_index;
  
        } else if (contrast < fast_max_contrast) {

          // 頂上を越えたのでフォーカス位置を二次関数近似で推定
          int est_focus_pos = second_order_approximation(fast_index-1, contrast2, contrast1, contrast0, fast_step) + fast_start_step;

          // モータをフォーカス推定位置に移動
          int move_step = est_focus_pos - focus_pos;
          FocusStepper.step(move_step);
          focus_pos = est_focus_pos;
          Serial.println("estimation focus position: " + String(focus_pos));

          // モードを次に進める
          mode = FINE_CONTRAST_AF;
          
          // 初期化フラグを立てる
          fast_init = true;
        }

      } else {

        // 頂上を見つけられなかったので、オートフォーカスをあきらめる
        Serial.println("Cannot find focus position. Give up");

        // モードをアイドルに戻す
        mode = IDLE_CONTRAST_AF;
        // ズームフォーカスレンズ制御終了
        digitalWrite(MOTOR_EN, LOW);

        // 初期化フラグを立てる
        fast_init = true;
      }
    } // fast_init

  } else if (mode == FINE_CONTRAST_AF) {

    static const int fine_rpm = 20;
    static const int range = 10;
    static bool fine_init = true;
    static int fine_start_step;
    static int fine_end_step;
    static int fine_max_contrast;

    if (fine_init) {

      fine_start_step = focus_pos - range;
      fine_end_step = focus_pos + range;
      fine_init = false; // 初期化済
      fine_max_contrast = 0;

      // フォーカスを走査スタート地点に設定する
      FocusStepper.setSpeed(fine_rpm);
      FocusStepper.step(-range);
      focus_pos -= range;

    } else {

       if (focus_pos <= fine_end_step && focus_pos >= fine_start_step) {

        if (contrast > fine_max_contrast) {

          // 頂上を越えなかったのでフォーカスを進める
          FocusStepper.step(1);
          fine_max_contrast = contrast;
          ++focus_pos;
 
        } else if (contrast < fine_max_contrast) {

          // 頂上を越えたのでフォーカス位置をひとつ戻す
          FocusStepper.step(-1);
          --focus_pos;
          Serial.println("fine focus position: " + String(focus_pos));

          // モードをアイドルに戻す
          mode = IDLE_CONTRAST_AF;
          // ズームフォーカスレンズ制御終了
          digitalWrite(MOTOR_EN, LOW);

          // 初期化フラグを立てる
          fine_init = true;

        }
      } else {
        // 頂上が見つけられなかったので諦める
        Serial.println("Cannot find focus position. Give up");

        // モードをアイドルに戻す
        mode = IDLE_CONTRAST_AF;
        // ズームフォーカスレンズ制御終了
        digitalWrite(MOTOR_EN, LOW);

        // 初期化フラグを立てる
        fine_init = true;
      }
    } // fine_init
 
  }
}


void setup() {
  Serial.begin(115200);
  pinMode(MOTOR_EN, OUTPUT);
  pinMode(intPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(intPin) ,changeState ,FALLING);

  tft.begin();
  tft.setRotation(3);

  while (!theSD.begin()) { 
    digitalWrite(LED3, HIGH); delay(100);
    Serial.println("Insert SD Card"); 
    digitalWrite(LED3, LOW); delay(100);
  } 

  digitalWrite(MOTOR_EN, HIGH);
  const int rpm = 50;
  FocusStepper.setSpeed(rpm);
  ZoomStepper.setSpeed(rpm);
  // ズームモータとフォーカスモータをホームポジションに移動
  FocusStepper.step(-focusMaxStep);
  ZoomStepper.step(-zoomMaxStep);
  digitalWrite(MOTOR_EN, LOW); 

  theCamera.begin(1, CAM_VIDEO_FPS_30, CAM_IMGSIZE_QQVGA_H, CAM_IMGSIZE_QQVGA_V, CAM_IMAGE_PIX_FMT_RGB565);
  theCamera.setStillPictureImageFormat(CAM_IMGSIZE_QUADVGA_H, CAM_IMGSIZE_QUADVGA_V, CAM_IMAGE_PIX_FMT_JPG, 6);
  // theCamera.setHDR(CAM_HDR_MODE_ON);
  theCamera.startStreaming(true, CamCB);
 
  digitalWrite(LED0, HIGH);
  delay(1000);

  last_time_ms = 0;
}


void loop() {

  if (bButtonPressed && mode == IDLE_CONTRAST_AF) {
    theCamera.startStreaming(false, CamCB);
    digitalWrite(LED2, HIGH);

    CamImage img = theCamera.takePicture(); 
    if (!img.isAvailable()) {
      Serial.println("take picture error");
      theCamera.startStreaming(true, CamCB);
      bButtonPressed = false;
      return;
    }

    static int g_counter = 0;
    char filename[16] = {0};
    sprintf(filename, "PICT%03d.jpg", g_counter++);
    if (theSD.exists(filename)) theSD.remove(filename);
    File myFile = theSD.open(filename, FILE_WRITE);
    myFile.write(img.getImgBuff(), img.getImgSize());
    myFile.close();
    Serial.println("Saved as " + String(filename));
  
    digitalWrite(LED2, LOW);
    theCamera.startStreaming(true, CamCB);
    bButtonPressed = false;
  }
}
