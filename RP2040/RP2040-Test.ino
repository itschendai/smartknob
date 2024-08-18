#include "LCD_Test.h"
#include <math.h>

#define signalPin 17
#define outputPin 16
volatile long StartTime = 0;
volatile long CurrentTime = 0;
volatile long Pulses = 0;

volatile int angle360 = 0;

int PulseWidth = 0;

int lastangle = 0;

int slide = 0;
int appswipe = 0;
int previousSlide = 0;

void Touch_INT_callback();
uint8_t flag = 0;
UWORD *BlackImage;
// const int adcPin = 26;
// int adcValue = analogRead(adcPin);
// String signal = "";
// float radius = 80; //pixels
void setup()
{
  Serial.begin(115200);
  pinMode(signalPin, INPUT_PULLUP);
  pinMode(outputPin, OUTPUT);
  digitalWrite(outputPin, LOW);
  attachInterrupt(digitalPinToInterrupt(signalPin),PulseTimer,CHANGE);

    // put your setup code here, to run once:
    if (DEV_Module_Init() != 0)
        Serial.println("GPIO Init Fail!");
    else
        Serial.println("GPIO Init successful!");
    LCD_1IN28_Init(HORIZONTAL);
    DEV_SET_PWM(0);
    LCD_1IN28_Clear(WHITE);
    DEV_SET_PWM(100);
    UDOUBLE Imagesize = LCD_1IN28_HEIGHT * LCD_1IN28_WIDTH * 2;
    if ((BlackImage = (UWORD *)malloc(Imagesize)) == NULL)
    {
        Serial.println("Failed to apply for black memory...");
        exit(0);
    }
    // /*1.Create a new image cache named IMAGE_RGB and fill it with white*/
    Paint_NewImage((UBYTE *)BlackImage, LCD_1IN28.WIDTH, LCD_1IN28.HEIGHT, 0, WHITE);
    Paint_SetScale(65);
    Paint_Clear(WHITE);
    Paint_SetRotate(ROTATE_0);
    Paint_Clear(WHITE);

    // /* GUI */
    Serial.println("drawing...\r\n");
    // /*2.Drawing on the image*/

    //startupSlide();
#if 1
    Paint_DrawPoint(50, 41, BLACK, DOT_PIXEL_1X1, DOT_FILL_RIGHTUP); // 240 240
    Paint_DrawPoint(50, 46, BLACK, DOT_PIXEL_2X2, DOT_FILL_RIGHTUP);
    Paint_DrawPoint(50, 51, BLACK, DOT_PIXEL_3X3, DOT_FILL_RIGHTUP);
    Paint_DrawPoint(50, 56, BLACK, DOT_PIXEL_4X4, DOT_FILL_RIGHTUP);
    Paint_DrawPoint(50, 61, BLACK, DOT_PIXEL_5X5, DOT_FILL_RIGHTUP);

    Paint_DrawLine(60, 40, 90, 70, MAGENTA, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
    Paint_DrawLine(60, 70, 90, 40, MAGENTA, DOT_PIXEL_2X2, LINE_STYLE_SOLID);

    Paint_DrawRectangle(60, 40, 90, 70, RED, DOT_PIXEL_2X2, DRAW_FILL_EMPTY);
    Paint_DrawRectangle(100, 40, 130, 70, BLUE, DOT_PIXEL_2X2, DRAW_FILL_FULL);

    Paint_DrawLine(135, 55, 165, 55, CYAN, DOT_PIXEL_1X1, LINE_STYLE_DOTTED);
    Paint_DrawLine(150, 40, 150, 70, CYAN, DOT_PIXEL_1X1, LINE_STYLE_DOTTED);

    Paint_DrawCircle(150, 55, 15, GREEN, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    Paint_DrawCircle(185, 55, 15, GREEN, DOT_PIXEL_1X1, DRAW_FILL_FULL);

    Paint_DrawNum(50, 80, 9.87654321, &Font20, 3, WHITE, BLACK);
    Paint_DrawString_EN(50, 100, "ABC", &Font20, 0x000f, 0xfff0);
    Paint_DrawString_CN(50, 120, "微雪电子", &Font24CN, WHITE, BLUE);
    Paint_DrawString_EN(50, 161, "WaveShare", &Font16, RED, WHITE);

    //Paint_DrawImage1(Signal816, 100, 100, 30, 30);

    // /*3.Refresh the picture in RAM to LCD*/
    LCD_1IN28_Display(BlackImage);
    DEV_Delay_ms(1000);
#endif


#if 0
    float acc[3], gyro[3];
    unsigned int tim_count = 0;
    uint16_t result;

    QMI8658_init();
    CST816S_init(CST816S_Gesture_Mode);
    DEV_KEY_Config(Touch_INT_PIN);
    attachInterrupt(Touch_INT_PIN, &Touch_INT_callback, RISING);
    Serial.println("QMI8658_init\r\n");
    DEV_SET_PWM(100);
    const float conversion_factor = 3.3f / (1 << 12) * 3;
    Paint_Clear(WHITE);
    // Paint_DrawRectangle(0, 00, 240, 47, 0XF410, DOT_PIXEL_2X2, DRAW_FILL_FULL);
    // Paint_DrawRectangle(0, 47, 240, 120, 0X4F30, DOT_PIXEL_2X2, DRAW_FILL_FULL);
    // Paint_DrawRectangle(0, 120, 240, 195, 0XAD55, DOT_PIXEL_2X2, DRAW_FILL_FULL);
    // Paint_DrawRectangle(0, 195, 240, 240, 0X2595, DOT_PIXEL_2X2, DRAW_FILL_FULL);

    // Paint_DrawString_EN(45, 30, "LongPress Quit", &Font16, BLACK, 0XF410);
    // Paint_DrawString_EN(45, 50, "ACC_X = ", &Font16, BLACK, 0X4F30);
    // Paint_DrawString_EN(45, 75, "ACC_Y = ", &Font16, BLACK, 0X4F30);
    // Paint_DrawString_EN(45, 100, "ACC_Z = ", &Font16, BLACK, 0X4F30);
    // Paint_DrawString_EN(45, 125, "GYR_X = ", &Font16, BLACK, 0XAD55);
    // Paint_DrawString_EN(45, 150, "GYR_Y = ", &Font16, BLACK, 0XAD55);
    // Paint_DrawString_EN(45, 175, "GYR_Z = ", &Font16, BLACK, 0XAD55);
    // Paint_DrawString_EN(45, 200, "BAT(V)=", &Font16, BLACK, 0X2595);
    LCD_1IN28_Display(BlackImage);
    while (true)
    {
        result = DEC_ADC_Read();
        QMI8658_read_xyz(acc, gyro, &tim_count);
        // Paint_Clear(WHITE);
        Paint_DrawRectangle(120, 47,  220, 120, 0X4F30, DOT_PIXEL_2X2, DRAW_FILL_FULL);
        Paint_DrawRectangle(120, 120, 220, 195, 0XAD55, DOT_PIXEL_2X2, DRAW_FILL_FULL);
        Paint_DrawRectangle(120, 195, 220, 240, 0X2595, DOT_PIXEL_2X2, DRAW_FILL_FULL);
        Paint_DrawNum(120, 50, acc[0], &Font16, 2, BLACK, 0X4F30);
        Paint_DrawNum(120, 75, acc[1], &Font16, 2, BLACK, 0X4F30);
        Paint_DrawNum(120, 100, acc[2], &Font16, 2, BLACK, 0X4F30);
        Paint_DrawNum(120, 125, gyro[0], &Font16, 2, BLACK, 0XAD55);
        Paint_DrawNum(120, 150, gyro[1], &Font16, 2, BLACK, 0XAD55);
        Paint_DrawNum(120, 175, gyro[2], &Font16, 2, BLACK, 0XAD55);

        Paint_DrawNum(130, 200, result * conversion_factor, &Font16, 2, BLACK, 0X2595);
        LCD_1IN28_DisplayWindows(120, 50, 210, 200, BlackImage);
        LCD_1IN28_DisplayWindows(130, 200, 220, 220, BlackImage);
        if (flag == 1)
        {
            flag = 0;
            break;
        }
    }
#endif
    
#if 0
    QMI8658_init();
    //CST816S_init(CST816S_Gesture_Mode);
    DEV_KEY_Config(Touch_INT_PIN);
    attachInterrupt(Touch_INT_PIN, &Touch_INT_callback, RISING);
    DEV_SET_PWM(100);
    Paint_Clear(WHITE);
    // Paint_DrawRectangle(0, 00, 240, 47, 0X2595, DOT_PIXEL_2X2, DRAW_FILL_FULL);
    // Paint_DrawString_EN(60, 30, "Touch test", &Font16, BLACK, 0X2595);
    LCD_1IN28_Display(BlackImage);
    CST816S_init(CST816S_Point_Mode);
    while (true)
    {
        if (flag == 1)
        {
            Paint_DrawPoint(Touch_CTS816.x_point, Touch_CTS816.y_point, BLACK, DOT_PIXEL_3X3, DOT_FILL_RIGHTUP);
            LCD_1IN28_DisplayWindows(Touch_CTS816.x_point, Touch_CTS816.y_point, Touch_CTS816.x_point + 3, Touch_CTS816.y_point + 3, BlackImage);
            Serial.printf("X:%d Y:%d\r\n", Touch_CTS816.x_point, Touch_CTS816.y_point);
            flag = 0;
        }
        DEV_Delay_us(500);
    }
#endif

#if 0
    Paint_Clear(BLACK);

    Paint_DrawString_EN(30, 80, "Initializing...", &Font20, WHITE, Black);

    delay(2);

    // /*3.Refresh the picture in RAM to LCD*/
    LCD_1IN28_Display(BlackImage);
#endif

#if 0
    float acc[3], gyro[3];
    unsigned int tim_count = 0;
    uint16_t result;

    QMI8658_init();
    CST816S_init(CST816S_Gesture_Mode);
    DEV_KEY_Config(Touch_INT_PIN);
    attachInterrupt(Touch_INT_PIN, &Touch_INT_callback, RISING);
    Serial.println("QMI8658_init\r\n");
    DEV_SET_PWM(100);
    const float conversion_factor = 3.3f / (1 << 12) * 3;
    Paint_Clear(WHITE);
    Paint_DrawRectangle(0, 00, 240, 47, 0XF410, DOT_PIXEL_2X2, DRAW_FILL_FULL);
    Paint_DrawRectangle(0, 47, 240, 120, 0X4F30, DOT_PIXEL_2X2, DRAW_FILL_FULL);
    Paint_DrawRectangle(0, 120, 240, 195, 0XAD55, DOT_PIXEL_2X2, DRAW_FILL_FULL);
    Paint_DrawRectangle(0, 195, 240, 240, 0X2595, DOT_PIXEL_2X2, DRAW_FILL_FULL);

    Paint_DrawString_EN(45, 30, "LongPress Quit", &Font16, BLACK, 0XF410);
    Paint_DrawString_EN(45, 50, "ACC_X = ", &Font16, BLACK, 0X4F30);
    Paint_DrawString_EN(45, 75, "ACC_Y = ", &Font16, BLACK, 0X4F30);
    Paint_DrawString_EN(45, 100, "ACC_Z = ", &Font16, BLACK, 0X4F30);
    Paint_DrawString_EN(45, 125, "GYR_X = ", &Font16, BLACK, 0XAD55);
    Paint_DrawString_EN(45, 150, "GYR_Y = ", &Font16, BLACK, 0XAD55);
    Paint_DrawString_EN(45, 175, "GYR_Z = ", &Font16, BLACK, 0XAD55);
    Paint_DrawString_EN(45, 200, "BAT(V)=", &Font16, BLACK, 0X2595);
    LCD_1IN28_Display(BlackImage);
    while (true)
    {
        result = DEC_ADC_Read();
        QMI8658_read_xyz(acc, gyro, &tim_count);
        Paint_Clear(WHITE);
        Paint_DrawRectangle(120, 47,  220, 120, 0X4F30, DOT_PIXEL_2X2, DRAW_FILL_FULL);
        Paint_DrawRectangle(120, 120, 220, 195, 0XAD55, DOT_PIXEL_2X2, DRAW_FILL_FULL);
        Paint_DrawRectangle(120, 195, 220, 240, 0X2595, DOT_PIXEL_2X2, DRAW_FILL_FULL);
        Paint_DrawNum(120, 50, acc[0], &Font16, 2, BLACK, 0X4F30);
        Paint_DrawNum(120, 75, acc[1], &Font16, 2, BLACK, 0X4F30);
        Paint_DrawNum(120, 100, acc[2], &Font16, 2, BLACK, 0X4F30);
        Paint_DrawNum(120, 125, gyro[0], &Font16, 2, BLACK, 0XAD55);
        Paint_DrawNum(120, 150, gyro[1], &Font16, 2, BLACK, 0XAD55);
        Paint_DrawNum(120, 175, gyro[2], &Font16, 2, BLACK, 0XAD55);

        Paint_DrawNum(130, 200, result * conversion_factor, &Font16, 2, BLACK, 0X2595);
        LCD_1IN28_DisplayWindows(120, 50, 210, 200, BlackImage);
        LCD_1IN28_DisplayWindows(130, 200, 220, 220, BlackImage);
        if (flag == 1)
        {
            flag = 0;
            break;
        }
    }
#endif

QMI8658_init();
DEV_KEY_Config(Touch_INT_PIN);
attachInterrupt(Touch_INT_PIN, &Touch_INT_callback, RISING);
DEV_SET_PWM(100);
Paint_Clear(WHITE);
// Paint_DrawRectangle(0, 00, 240, 47, 0X2595, DOT_PIXEL_2X2, DRAW_FILL_FULL);
// Paint_DrawString_EN(60, 30, "Touch test", &Font16, BLACK, 0X2595);
//LCD_1IN28_Display(BlackImage);
CST816S_init(CST816S_Gesture_Mode);
//CST816S_init(CST816S_Point_Mode);
Paint_Clear(BLACK);

}

void loop()
{
  // if (Serial.available() > 0) {
  //       String incoming = Serial.readStringUntil('\n');
  //       Serial.println("Received: " + incoming);
  //       Paint_Clear(WHITE);

  //       Paint_DrawString_EN(30, 80, "HELLO", &Font20, BLACK, WHITE);
  //       Paint_DrawString_EN(30, 112, incoming.c_str(), &Font16, BLACK, WHITE);

  //       // /*3.Refresh the picture in RAM to LCD*/
  //       LCD_1IN28_Display(BlackImage);
  //   }

  // adcValue = analogRead(adcPin);
  // //signal = String((adcValue/4095.0*277.8)-10);

  // float radian = convertSignalToRadians((adcValue/4095.0*277.8)-10);
  // DrawCircleWRad(radian);
  if (slide == 1) {
    //Paint_DrawLine(50, 120, 190, 120, 0x07FF, DOT_PIXEL_2X2, LINE_STYLE_DOTTED);
    Paint_DrawLine(120, 30, 120, 92, 0x07FF, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
  }
  if (slide == 2) {
    Paint_DrawLine(75, 42, 106, 96, 0x07FF, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
    Paint_DrawLine(165, 42, 134, 96, 0x07FF, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
    if (angle360 > 0 && angle360 < 60) {
      Paint_DrawString_EN(100, 140, "ON", &FontMenlo32, 0x07FF, BLACK);
    }
    else if (angle360 < 360 && angle360 > 300) {
      Paint_DrawString_EN(90, 140, "OFF", &FontMenlo32, 0x07FF, BLACK);
    }
  }

  if (slide == 3) {
    Paint_DrawLine(30, 120, 92, 120, 0x07FF, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
    Paint_DrawLine(148, 120, 210, 120, 0x07FF, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
  }

  if (slide == 4) {
    Paint_DrawLine(30, 120, 92, 120, 0x07FF, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
    Paint_DrawLine(148, 120, 210, 120, 0x07FF, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
    Paint_DrawLine(120, 30, 120, 92, 0x07FF, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
    Paint_DrawLine(120, 148, 120, 210, 0x07FF, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
  }

  if (slide == 5) {
    Paint_Clear(BLACK);
    Paint_DrawString_EN(45, 105, "SPIN IT!", &FontMenlo32, 0x07FF, BLACK);
    LCD_1IN28_Display(BlackImage);
  }

  if (slide < 5) {
    DrawCircleWRad((PulseWidth-5)*0.0174533);
  }

  if (slide == 6) {
    DrawVolumeWRad((PulseWidth-5)*0.0174533);
  }

  if (slide == 7) {
    Paint_DrawString_EN(92, 92, "ALT", &FontMenlo32, 0x07FF, BLACK);
    //Paint_DrawString_EN(92, 100, "+", &FontMenlo32, 0xC618, BLACK);
    Paint_DrawString_EN(92, 130, "TAB", &FontMenlo32, 0x07FF, BLACK);
    Paint_DrawLine(120, 30, 120, 80, 0xC618, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
    Paint_DrawLine(165, 42, 140, 85, 0xFA00, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
    DrawCircleWRad((PulseWidth-5)*0.0174533);
    //LCD_1IN28_Display(BlackImage);
  }

  if (slide == 8) {
    Paint_Clear(BLACK);
    if (appswipe == 1){
      Paint_DrawImage(Signal816, 45, 45, 150, 150);
    }
    if (appswipe == 2){
      Paint_DrawImage(Bat816, 45, 45, 150, 150);
    }
    if (appswipe == 0){
      Paint_DrawImage(Msg816, 45, 45, 150, 150);
    }
    LCD_1IN28_Display(BlackImage);
    //DEV_Delay_ms(10000);
  }

  if (slide == 9) {
    slide = 0;
  }


  //Serial.println(PulseWidth);

  // Paint_Clear(WHITE);

  // Paint_DrawString_EN(30, 80, "HELLO", &Font20, BLACK, WHITE);
  // Paint_DrawString_EN(30, 112, signal.c_str(), &Font16, BLACK, WHITE);

  //       // /*3.Refresh the picture in RAM to LCD*/
  // LCD_1IN28_Display(BlackImage);
  //Serial.println(PulseWidth);
  //DEV_Delay_ms(0);
}
void Touch_INT_callback()
{

    if (Touch_CTS816.mode == CST816S_Gesture_Mode)
    {
        uint8_t gesture = CST816S_Get_Gesture();
        if (gesture == CST816S_Gesture_Left)
        {
            Serial.println("Swipe Left");
            slide++;
            togglePin(1);
        }
        if (gesture == CST816S_Gesture_Right)
        {
            Serial.println("Swipe Right");
            slide--;
            togglePin(2);
        }
        if (gesture == CST816S_Gesture_Up)
        {
            if (slide == 8){
              appswipe ++;
              if (appswipe == 3){
                appswipe = 0;
              }
            }
            else{
              togglePin(1);
            }
            Serial.println("Swipe Up");
            
        }
        if (gesture == CST816S_Gesture_Down)
        {
            Serial.println("Swipe Down");
            togglePin(2);
        }
    }
    else
    {
        CST816S_Get_Point();
        Serial.println("Points");
    }
}

void DrawCircleWRad(float rad) {
    int centerX = 120; // X coordinate of the circle's center
    int centerY = 120; // Y coordinate of the circle's center
    int radius = 110;   // Radius of the path on which the circle will rotate
    angle360 = rad/6.26*360;
    if (angle360 == 360){
      angle360 = 0;
    }

    // Calculate X and Y using the angle 'rad'
    int X = centerX + radius * cos(rad - M_PI / 2);
    int Y = centerY + radius * sin(rad - M_PI / 2);

    // Clear the display
    //Paint_Clear(BLACK);
    
    // Draw a circle at the calculated coordinates with a fixed size of 15
    //Paint_DrawLine(50, 120, 190, 120, BLUE, DOT_PIXEL_2X2, LINE_STYLE_DOTTED);
    //Paint_DrawLine(120, 50, 120, 190, BLUE, DOT_PIXEL_2X2, LINE_STYLE_DOTTED);

    if (slide != 7){
      if ((String(angle360)).length() == 1) {
            Paint_DrawString_EN(110, 100, String(angle360).c_str(), &FontMenlo32, 0xC618, BLACK);
      }
      if ((String(angle360)).length() == 2) {
            Paint_DrawString_EN(100, 100, String(angle360).c_str(), &FontMenlo32, 0xC618, BLACK);
      }
      if ((String(angle360)).length() == 3) {
            Paint_DrawString_EN(92, 100, String(angle360).c_str(), &FontMenlo32, 0xC618, BLACK);
      }
      Paint_DrawCircle(X, Y, 8, 0xFA00, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    }
    else {
      Paint_DrawCircle(X, Y, 8, 0xC618, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    }
    
    // Refresh the display
    LCD_1IN28_Display(BlackImage);
}

void DrawVolumeWRad(float rad) {
    int centerX = 120; // X coordinate of the circle's center
    int centerY = 120; // Y coordinate of the circle's center
    int radius = 110;   // Radius of the path on which the circle will rotate
    angle360 = rad/6.26*360;
    // if (angle360 == 360){
    //   angle360 = 0;
    // }

    if ((lastangle >= 300 && angle360 < 180) || (lastangle < 60 && angle360 >= 180)) {
        // Do nothing
        return;
    }
    else {
      for (float theta = 0; theta <= rad; theta += 0.01) {
      // Calculate X and Y using the angle 'rad'
      int X = centerX + radius * cos(theta - M_PI / 2);
      int Y = centerY + radius * sin(theta - M_PI / 2);

      Paint_DrawCircle(X, Y, 8, 0xFA00, DOT_PIXEL_1X1, DRAW_FILL_FULL);
      }

    int percentage = angle360/3.6;

    // Convert percentage to string
    String percentageString = String(percentage) + "%";

    Paint_DrawString_EN(60, 75, String("VOLUME").c_str(), &FontMenlo32, 0xC618, BLACK);

    // Display the percentage value on the screen with appropriate alignment
    if (percentageString.length() == 2) {
        Paint_DrawString_EN(100, 120, percentageString.c_str(), &FontMenlo32, 0x07FF, BLACK);
    } else if (percentageString.length() == 3) {
        Paint_DrawString_EN(90, 120, percentageString.c_str(), &FontMenlo32, 0x07FF, BLACK);
    } else if (percentageString.length() == 4) { // For 100%
        Paint_DrawString_EN(80, 120, percentageString.c_str(), &FontMenlo32, 0x07FF, BLACK);
    }
    
    // Refresh the display
    lastangle = angle360;

    }
    
    LCD_1IN28_Display(BlackImage);
}


void PulseTimer(){
  CurrentTime = micros();
  if (CurrentTime > StartTime){
    Pulses = CurrentTime - StartTime;
    StartTime = CurrentTime;
  }
  if (Pulses < 366){
    PulseWidth = Pulses;
  }
}

void togglePin(int x) {
    for (int i = 0; i < x; i++) {
        digitalWrite(outputPin, HIGH);  // Turn pin 17 on
        delay(1);                // Wait for 5 milliseconds
        digitalWrite(outputPin, LOW);   // Turn pin 17 off
    }
}