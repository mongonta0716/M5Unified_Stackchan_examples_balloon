#include <Arduino.h>
#include <Avatar.h>
#include <Stackchan_servo.h>
#include <Stackchan_system_config.h>
#include <Stackchan_Takao_Base.hpp>
#include <formatString.hpp>
#include <SD.h>

using namespace m5avatar;

#define USE_SERVO

#if defined( ARDUINO_M5STACK_CORES3 )
  // M5Stack CoreS3用の設定 ※暫定的にplatformio.iniにARDUINO_M5STACK_CORES3を定義しています。
  // Port.A X:G1 Y:G2
  // Port.B X:G8 Y:G9
  // Port.C X:18 Y:17
  #include <gob_unifiedButton.hpp> // 2023/5/12現在 M5UnifiedにBtnA等がないのでGobさんのライブラリを使用
  goblib::UnifiedButton unifiedButton;
#endif

Avatar avatar;
StackchanSERVO servo;
float mouth_ratio = 0.0f;

fs::FS yaml_fs = SD; // JSONファイルの収納場所(SPIFFS or SD)
StackchanSystemConfig system_config;
const char* stackchan_system_config_yaml = "/yaml/SC_Config.yaml";

/// set M5Speaker virtual channel (0-7)
static constexpr uint8_t m5spk_virtual_channel = 0;


const Expression expressions_table[] = {
  Expression::Neutral,
  Expression::Happy,
  Expression::Sleepy,
  Expression::Doubt,
  Expression::Sad,
  Expression::Angry
};
void servoLoop(void *args) {
  DriveContext *ctx = (DriveContext *)args;
  Avatar *avatar = ctx->getAvatar();
  long move_time = 0;
  long interval_time = 0;
  long move_x = 0;
  long move_y = 0;
  float gaze_x = 0.0f;
  float gaze_y = 0.0f;
  bool sing_mode = false;
  while (avatar->isDrawing()) {
    if (mouth_ratio == 0.0f) {
      // 待機時の動き
      interval_time = random(system_config.getServoInterval(AvatarMode::NORMAL)->interval_min
                           , system_config.getServoInterval(AvatarMode::NORMAL)->interval_max);
      move_time = random(system_config.getServoInterval(AvatarMode::NORMAL)->move_min
                       , system_config.getServoInterval(AvatarMode::NORMAL)->move_max);
      sing_mode = false;

    } else {
      // 歌うモードの動き
      interval_time = random(system_config.getServoInterval(AvatarMode::SINGING)->interval_min
                           , system_config.getServoInterval(AvatarMode::SINGING)->interval_max);
      move_time = random(system_config.getServoInterval(AvatarMode::SINGING)->move_min
                       , system_config.getServoInterval(AvatarMode::SINGING)->move_max);
      sing_mode = true;
    } 
    avatar->getGaze(&gaze_y, &gaze_x);
    
//    Serial.printf("x:%f:y:%f\n", gaze_x, gaze_y);
    // X軸は90°から+-で左右にスイング
    if (gaze_x < 0) {
      move_x = system_config.getServoInfo(AXIS_X)->start_degree - mouth_ratio * 15 + (int)(30.0 * gaze_x);
    } else {
      move_x = system_config.getServoInfo(AXIS_X)->start_degree + mouth_ratio * 15 + (int)(30.0 * gaze_x);
    }
    // Y軸は90°から上にスイング（最大35°）
    move_y = system_config.getServoInfo(AXIS_Y)->start_degree - mouth_ratio * 10 - abs(25.0 * gaze_y);
    servo.moveXY(move_x, move_y, move_time);
    vTaskDelay(interval_time/portTICK_PERIOD_MS);
  }
  vTaskDelete(NULL);
}


void setup() {
  auto cfg = M5.config();
  cfg.output_power = true;

  M5.begin(cfg);
#if defined( ARDUINO_M5STACK_CORES3 )
  unifiedButton.begin(&M5.Display, goblib::UnifiedButton::appearance_t::transparent_all);
#endif

  { /// custom setting
    auto spk_cfg = M5.Speaker.config();
    /// Increasing the sample_rate will improve the sound quality instead of increasing the CPU load.
    // M5Stack Fire/Core2/AWS 向けPSRAM搭載機種のパラメータ
    spk_cfg.sample_rate = 96000; // default:64000 (64kHz)  e.g. 48000 , 50000 , 80000 , 96000 , 100000 , 128000 , 144000 , 192000 , 200000
    spk_cfg.task_pinned_core = APP_CPU_NUM;
    // spk_cfg.task_priority = configMAX_PRIORITIES - 2;
    spk_cfg.dma_buf_count = 20;
    //spk_cfg.stereo = true;
    spk_cfg.dma_buf_len = 256;
    M5.Speaker.config(spk_cfg);
  }


  M5.Speaker.begin();

  // BASICとFIREのV2.6で25MHzだと読み込めないため15MHzまで下げています。
  SD.begin(GPIO_NUM_4, SPI, 15000000);
  
  delay(1000);
  system_config.loadConfig(yaml_fs, stackchan_system_config_yaml);

  M5.Speaker.setVolume(system_config.getBluetoothSetting()->start_volume);
  M5.Speaker.setChannelVolume(system_config.getBluetoothSetting()->start_volume, m5spk_virtual_channel);

  if (system_config.getUseTakaoBase()) {
    checkTakaoBasePowerStatus(&M5.Power, &servo);
    M5.Power.setExtOutput(false);
  }


  if ((system_config.getServoInfo(AXIS_X)->pin == 21)
     || (system_config.getServoInfo(AXIS_X)->pin == 22)) {
    // Port.Aを利用する場合は、I2Cが使えないのでアイコンは表示しない。
    avatar.setBatteryIcon(false);
    if (M5.getBoard() == m5::board_t::board_M5Stack) {
      M5.In_I2C.release();
    }
  } else {
    avatar.setBatteryIcon(true);
    avatar.setBatteryStatus(M5.Power.isCharging(), M5.Power.getBatteryLevel());
  }
  
  servo.begin(system_config.getServoInfo(AXIS_X)->pin, system_config.getServoInfo(AXIS_X)->start_degree,
              system_config.getServoInfo(AXIS_X)->offset,
              system_config.getServoInfo(AXIS_Y)->pin, system_config.getServoInfo(AXIS_Y)->start_degree,
              system_config.getServoInfo(AXIS_Y)->offset,
              (ServoType)system_config.getServoType());
  delay(2000);

  avatar.init(1); // start drawing
  avatar.addTask(servoLoop, "servoLoop", 2048U, 1);
  avatar.setExpression(Expression::Neutral);
  avatar.setSpeechFont(system_config.getFont());

}

void loop() {
#ifdef ARDUINO_M5STACK_CORES3
  unifiedButton.update(); // To be called before M5.update.
#endif

  M5.update(); // Button status change detection, needs to be performed before button status is detected
  if (M5.BtnA.wasClicked()) {
    avatar.setSpeechText("BtnA Pressed");
  }
  if (M5.BtnB.wasClicked()) {
    avatar.setSpeechText("BtnB Pressed");
  }
  if (M5.BtnC.wasClicked()) {
    avatar.setSpeechText("BtnC Pressed");
  }

}