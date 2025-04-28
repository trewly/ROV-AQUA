#define PWM_PIN1 13
#define PWM_CHANNEL1 0

#define PWM_PIN2 12
#define PWM_CHANNEL2 1

#define PWM_PIN3 14
#define PWM_CHANNEL3 2

#define PWM_PIN4 27
#define PWM_CHANNEL4 3

#define PWM_FREQ 500
#define PWM_RESOLUTION 8

#define UART_TX 17
#define UART_RX 16

int maxSpeedForward = 100;
int maxSpeedBackward = -100;
int maxSpeedDive = -100;
int maxSpeedSurface = 100;
int SpeedLeft = 0;
int SpeedRight = 0;

int mapToPWM(int percent) {
  return map(percent, -100, 100, 128, 255);
}

void setup() {
  Serial.begin(115200);

  Serial2.begin(9600, SERIAL_8N1, UART_RX, UART_TX);
  Serial.println("UART1 started");

  // Thiết lập PWM cho các kênh
  ledcSetup(PWM_CHANNEL1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWM_PIN1, PWM_CHANNEL1);

  ledcSetup(PWM_CHANNEL2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWM_PIN2, PWM_CHANNEL2);

  ledcSetup(PWM_CHANNEL3, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWM_PIN3, PWM_CHANNEL3);

  ledcSetup(PWM_CHANNEL4, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWM_PIN4, PWM_CHANNEL4);

  // Viết giá trị PWM ban đầu
  ledcWrite(PWM_CHANNEL1, mapToPWM(75));
  ledcWrite(PWM_CHANNEL2, mapToPWM(75));
  ledcWrite(PWM_CHANNEL3, mapToPWM(75));
  ledcWrite(PWM_CHANNEL4, mapToPWM(75));

  delay(2000);
  Serial.println("PWM signals initialized.");
}

void loop() {
  if (Serial2.available()) {
    Serial.println("Data available on UART2");
    String data = Serial2.readStringUntil('\n');
    Serial.println("Raw data: " + data);

    int separatorIndex = data.indexOf(':');
    if (separatorIndex != -1) {
      String commandStr = data.substring(0, separatorIndex);
      String paramStr = data.substring(separatorIndex + 1);

      int command = commandStr.toInt();
      int param = paramStr.toInt();

      Serial.print("Command (int): ");
      Serial.println(command);
      Serial.print("Param (int): ");
      Serial.println(param);

      // Gọi hàm xử lý lệnh
      handleCommand(command, param);
    } else {
      Serial.println("Invalid format. Expected 'command:param'");
    }
  }
}

void handleCommand(int command, int param) {
  switch (command) {
    case 13: // SET_SPEED_FORWARD
      maxSpeedForward = constrain(param, -100, 100);
      Serial.print("Set max speed forward: ");
      Serial.println(maxSpeedForward);
      break;

    case 14: // SET_SPEED_BACKWARD
      maxSpeedBackward = constrain(param, -100, 100);
      Serial.print("Set max speed backward: ");
      Serial.println(maxSpeedBackward);
      break;

    case 8: // LEFT
      maxSpeedLeft = constrain(param, -100, 100);
      Serial.print("Set max speed left: ");
      Serial.println(maxSpeedLeft);
      break;

    case 9: // RIGHT
      maxSpeedRight = constrain(param, -100, 100);
      Serial.print("Set max speed right: ");
      Serial.println(maxSpeedRight);
      break;

    case 15: // SET_SPEED_SURFACE
      maxSpeedSurface = constrain(param, -100, 100);
      Serial.print("Set max speed surface: ");
      Serial.println(maxSpeedSurface);
      break;

    case 16: // SET_SPEED_DIVE
      maxSpeedDive = constrain(param, -100, 100);
      Serial.print("Set max speed dive: ");
      Serial.println(maxSpeedDive);
      break;

    case 6: // FORWARD
      ledcWrite(PWM_CHANNEL1, mapToPWM(maxSpeedForward));
      ledcWrite(PWM_CHANNEL2, mapToPWM(maxSpeedForward));
      Serial.println("Moving forward.");
      break;

    case 7: // BACKWARD
      ledcWrite(PWM_CHANNEL1, mapToPWM(maxSpeedBackward));
      ledcWrite(PWM_CHANNEL2, mapToPWM(maxSpeedBackward));
      Serial.println("Moving backward.");
      break;

    case 10: // SURFACE
      ledcWrite(PWM_CHANNEL3, mapToPWM(maxSpeedSurface));
      ledcWrite(PWM_CHANNEL4, mapToPWM(maxSpeedSurface));
      Serial.println("Surfacing.");
      break;

    case 11: // DIVE
      ledcWrite(PWM_CHANNEL3, mapToPWM(maxSpeedDive));
      ledcWrite(PWM_CHANNEL4, mapToPWM(maxSpeedDive));
      Serial.println("Diving.");
      break;

    default:
      Serial.println("Unknown command received.");
      break;
  }
}
