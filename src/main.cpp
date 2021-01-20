#include <Arduino.h>
#include <NeoPixelBus.h>
#include <NeoPixelAnimator.h>
#include <SPI.h>
#include <SD.h>

// Strip constants
const uint16_t PixelCount = 73;       // make sure to set this to the number of pixels in your strip
const uint8_t PixelPin = 6;           // make sure to set this to the correct pin, ignored for Esp8266

// Encoder constants
const uint8_t Switch = 2;
const uint8_t Clock = 3;
volatile const uint8_t Data = 4;

// Support constants
const uint8_t ChipSelect = 10;
const uint8_t NumModes = 4;
const uint8_t NumSubModes = 2;
const uint8_t NumStates = 2;

// Light constants
const uint16_t AnimCount = (PixelCount / 5 * 2 + 1) * 2; // need enough animation channels for both tails of the wipe functions
const uint16_t PathStart = 21;
const uint16_t PathLength = PixelCount / 2;

struct animationState {
  HsbColor startColor;
  HsbColor endColor;
  uint16_t indexPixel;
};

//function prototypes
uint8_t incrementMode(uint8_t, uint8_t, uint8_t);
int checkButton();
void solid();
void cycleSolid();
void wipeToBlack();
void wipeToColor();

NeoPixelBus<NeoGrbFeature, NeoNrf52xPwm0800KbpsMethod> strip(PixelCount, PixelPin);
NeoPixelAnimator animations(AnimCount);
NeoGamma<NeoGammaTableMethod> colorGamma;

// Support variables
volatile unsigned long newParamsSet = 0;
File paramFile;
float params[20];

// Light function variables
float standby = 0;                                  // placeholder variable used where rotate() function should do nothing
uint8_t mode = 0;                                   
uint8_t subMode = 0;                                // changes the value of adjustFactor
volatile float modeLimit = 1.0;                     // value at which adjustValue should wrap back to 0, resetting adjustValue to the bottom of it's range
volatile uint8_t adjustState = 0;                   // another submode that changes the value of adjustFactor
volatile float adjustFactor = 0.1;                  // amount by which adjustValue will change when knob is turned **this value is dependent on the current mode and subMode**
volatile float *adjustValue;                        // used to dynamically adjust settings specific to the current mode/submode 
HsbColor currentColor = HsbColor(0.0, 1.0, 0.5);    // current color of active leds
HsbColor solidColor = HsbColor(0.0, 1.0, 0.5);      // stored solid color used to set currentColor after mode changes
HsbColor wipeColor = HsbColor(0.0, 1.0, 0.5);       // same use as solidColor
animationState animProps[AnimCount];                // array of structs that hold requisite data for animation channels
uint16_t nextPixelMoveDuration = 2000 / PixelCount; // how fast we move through the pixels
uint16_t pixelFadeDuration = 500;                   // this/nextPixelMoveduration = length of tail; Length of tail * 2 must be < number of anim channels
typedef void (*OptionMenu[])();
OptionMenu modeList = {
  solid, 
  cycleSolid,
  wipeToBlack,
  wipeToColor
};

// Light function timing variables
float cycleSolidDelay = 2000;

// Button timing variables
volatile uint8_t debounce = 20; // ms debounce period to prevent flickering when pressing or releasing the button
uint16_t DCgap = 250;           // max ms between clicks for a double click event
uint16_t holdTime = 1000;       // ms hold period: how long to wait for press+hold event
uint16_t longHoldTime = 3000;   // ms long hold period: how long to wait for press+hold event

// Button variables
boolean buttonVal = HIGH;           // value read from button
boolean buttonLast = HIGH;          // buffered value of the button's previous state
boolean DCwaiting = false;          // whether we're waiting for a double click (down)
boolean DConUp = false;             // whether to register a double click on next release, or whether to wait and click
boolean singleOK = true;            // whether it's OK to do a single click
long downTime = -1;                 // time the button was pressed down
long upTime = -1;                   // time the button was released
boolean ignoreUp = false;           // whether to ignore the button release because the click+hold was triggered
boolean waitForUp = false;          // when held, whether to wait for the up event
boolean holdEventPast = false;      // whether or not the hold event happened already
boolean longHoldEventPast = false;  // whether or not the long hold event happened already

//=================================================
// Events to trigger

void clickEvent() { // cycle to next mode
  Serial.println("CLICK");
  mode = incrementMode(mode, 1, NumModes);
  switch (mode) {
    case 0: // static solid color
      currentColor = solidColor;
      adjustValue = &currentColor.H;
      adjustFactor = 0.01;
      break;
    case 1: // cycle solid color
      solidColor = currentColor; // save the current solid color so we can load it when we return to that mode
      adjustValue = &cycleSolidDelay;
      adjustFactor = 100;
      modeLimit = 10000;
      break;
    case 2: // fade to black wipe (twin cylon-esque)
      currentColor = wipeColor;
      adjustValue = &currentColor.H;
      adjustFactor = 0.01;
      modeLimit = 1.0;
      break;
    case 3: // cycle wipe 
      wipeColor = currentColor;
      adjustValue = &standby;
      break;
    default:
      break;
  }
  adjustState = 0;
  newParamsSet = millis();
}

void doubleClickEvent() { // cycle between coarse vs fine adjustment
  Serial.println("DOUBLE CLICK");
  adjustState = incrementMode(adjustState, 1, NumStates);
  switch (adjustState) {
    case 0:
      adjustFactor /= 10;
      break;
    case 1:
      adjustFactor *= 10;
      break;
  }
  newParamsSet = millis();
}

void holdEvent() { // cycle between color vs brightness adjustment
  Serial.println("SHORT HOLD");
  subMode = incrementMode(subMode, 1, NumSubModes);
  switch (subMode) {
    case 0:
      break;
    case 1:
      break;
  }
  newParamsSet = millis();
}

void longHoldEvent() { // reset all values to default
  Serial.println("LONG HOLD");
  mode = 0;
  subMode = 0;
  currentColor = HsbColor(0.0, 1.0, 0.5);
  solidColor = HsbColor(0.0, 1.0, 0.5);  
  wipeColor = HsbColor(0.0, 1.0, 0.5);   
  newParamsSet = millis();
  adjustValue = &currentColor.H;
  adjustFactor = 0.01;
  adjustState = 0;
  modeLimit = 1.0;
  cycleSolidDelay = 2000;
}

void rotate() {
  volatile static unsigned long lastInterruptTime = 0;
  volatile unsigned long timeAtInterrupt = millis();
  if (timeAtInterrupt - lastInterruptTime > debounce) {
    if (digitalRead(Data) == HIGH) {
      *adjustValue = fmod(*adjustValue + (modeLimit - adjustFactor), modeLimit);
    } else {
      *adjustValue = fmod(*adjustValue + adjustFactor, modeLimit);
    }
    lastInterruptTime = timeAtInterrupt;
    newParamsSet = millis();
  }
}

//================================================================================
// Animations

void BlendAnimUpdate(const AnimationParam& param) {
  RgbColor updatedColor = RgbColor::LinearBlend(
    animProps[0].startColor,
    animProps[0].endColor,
    param.progress
  );
  for (uint16_t pixel = 0; pixel < PixelCount; pixel++) {
    strip.SetPixelColor(pixel, updatedColor);
  }
}

void FadeToAnimUpdate(const AnimationParam& param) {  
  // RgbColor updatedColor = HsbColor::LinearBlend<NeoHueBlendShortestDistance>(
  RgbColor updatedColor = RgbColor::LinearBlend(
    animProps[param.index].startColor,
    animProps[param.index].endColor,
    param.progress);
  // apply the color to the strip
  strip.SetPixelColor(animProps[param.index].indexPixel, 
    colorGamma.Correct(updatedColor));
}

//================================================================================
// Light functions

void solid() {
  for (uint8_t pixel = 0; pixel < PixelCount; pixel++) {
    strip.SetPixelColor(pixel, currentColor);
  }
}

void cycleSolid() {
  if (!animations.IsAnimating()) {
    animProps[0].startColor = currentColor;
    animProps[0].endColor = HsbColor(currentColor.H + 0.1, currentColor.S, currentColor.B);
    animations.StartAnimation(0, (uint16_t)cycleSolidDelay, BlendAnimUpdate);
    currentColor.H += 0.1;
    if (currentColor.H > 1.0) {
      currentColor.H = 0.0;
    }
  }
}

void wipeTo(bool aColor) {
  static long lastCycleStart = millis();
  long currentTime = millis();
  static uint16_t pixelPath = 0;
  static uint16_t forwardFrontPixel = PathStart;
  static uint16_t reverseFrontPixel = PathStart;
  // this is where the delay that determines speed is enforced 
  if (currentTime - lastCycleStart >= nextPixelMoveDuration) {
    lastCycleStart = currentTime;
    // pick the next pixel inline to start animating
    forwardFrontPixel = (forwardFrontPixel + 1) % PixelCount; // increment 
    reverseFrontPixel = (reverseFrontPixel + (PixelCount - 1)) % PixelCount;
    pixelPath++;
    if (pixelPath > PathLength)
    {
      // we looped, lets pick a new front color
      if (aColor) {
        currentColor.H = fmod(currentColor.H + 0.2, 1.0);
      }
      forwardFrontPixel = PathStart;
      reverseFrontPixel = PathStart;
      pixelPath = 0;
    }
    uint16_t indexAnim;
    // do we have an animation available to use to animate the next front pixel?
    // if you see skipping, then either you are going to fast or need to increase
    // the number of animation channels

    if (animations.NextAvailableAnimation(&indexAnim, 0))
    {
      animProps[indexAnim].startColor = currentColor;
      animProps[indexAnim].endColor = aColor ? HsbColor(fmod(currentColor.H + 0.2, 1.0), currentColor.S, currentColor.B) : HsbColor(0.0, 0.0, 0.0);
      animProps[indexAnim].indexPixel = forwardFrontPixel;

      animations.StartAnimation(indexAnim, pixelFadeDuration, FadeToAnimUpdate);
    }
    if (animations.NextAvailableAnimation(&indexAnim, 0))
    {
      animProps[indexAnim].startColor = currentColor;
      animProps[indexAnim].endColor = aColor ? HsbColor(fmod(currentColor.H + 0.2, 1.0), currentColor.S, currentColor.B) : HsbColor(0.0, 0.0, 0.0);
      animProps[indexAnim].indexPixel = reverseFrontPixel;

      animations.StartAnimation(indexAnim, pixelFadeDuration, FadeToAnimUpdate);
    }
  }
}

void wipeToBlack() {
  wipeTo(false);
}

void wipeToColor() {
  wipeTo(true);
}

//================================================================================
// Support functions

uint8_t incrementMode(uint8_t a, uint8_t b, uint8_t m) {
  a += b;
  while (a >= m)
    a -= m;

  return a;
}

void readParams() {
  char digits[10];
  bool receiving = false;
  uint8_t index = 0;
  uint8_t next_param = 0;
  char *ptr;
  while (paramFile.available()) {
    char next_char = paramFile.read();
    Serial.print(next_char);
    if (receiving == true) {
      if (next_char == '>') {
        digits[index] = '\0';
        params[next_param] = strtod(digits, &ptr);
        Serial.println();
        Serial.print("new param: ");
        Serial.println(params[next_param]);
        next_param++;
        index = 0;
        receiving = false;
      } else {
        digits[index] = next_char;
        index++;
      }
    } else if (next_char == '<') {
      receiving = true;
    }
  }
  params[next_param] = '\0';
}

boolean writeParams(char newSettings[]) {
  SD.remove("params.txt");
  paramFile = SD.open("params.txt", FILE_WRITE); // filename must be 8 characters or less
  if (paramFile) {
    for (uint8_t i = 0; i < strlen(newSettings); i++) {
      paramFile.print(newSettings[i]);
    }
  } else {
    return false;
  }
  paramFile.close();
  return true;
}

void initSD() {
  Serial.print("Initializing card... ");
  if (!SD.begin(ChipSelect)) {
    Serial.println("Card failed or not present!");
  } else {
    Serial.println("Card initialized!");
    // SD.remove("params.txt");
    paramFile = SD.open("params.txt");
    if (!paramFile) {
      Serial.print("No parameter file exists. Creating now... ");
      char defaultSettings[256] = "Mode: <0>\n"
        "currentColor {Hue: <0.00> | Saturation: <1.00> | Brightness: <0.50>}\n"
        "solidColor {Hue: <0.00> | Saturation: <1.00> | Brightness: <0.50>}\n"
        "wipeColor {Hue: <0.00> | Saturation: <1.00> | Brightness: <0.50>}\n";
      if (writeParams(defaultSettings)) {
        Serial.println("File created successfully!");
      } else {
        Serial.println("File creation failed!");
      }
    } else {
      Serial.println("Parameter file found. Loading... ");
      readParams();
      paramFile.close();
      for (uint8_t i = 0; i < (sizeof(params) / sizeof(params[0])); i++) {
        Serial.print("Parameter ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.println(params[i]);
      }
      // load last saved settings
      mode = (int) params[0];
      currentColor.H = params[1];
      currentColor.S = params[2];
      currentColor.B = params[3];
      solidColor.H = params[4];
      solidColor.S = params[5];
      solidColor.B = params[6];
      wipeColor.H = params[7];
      wipeColor.S = params[8];
      wipeColor.B = params[9];
      Serial.println(mode);
      Serial.println(currentColor.H);
      Serial.println(currentColor.S);
      Serial.println(currentColor.B);
      Serial.println(solidColor.H);
      Serial.println(solidColor.S);
      Serial.println(solidColor.B);
      Serial.println(wipeColor.H);
      Serial.println(wipeColor.S);
      Serial.println(wipeColor.B);
    }
  }
}
//================================================================================
// System functions

void setup() {
  delay(3000); // safe reset delay (this can be removed once the program is stable)

  // Set button/encoder input pins
  pinMode(Switch, INPUT_PULLUP);
  pinMode(Clock, INPUT);
  pinMode(Data, INPUT);
  pinMode(PixelPin, OUTPUT);

  // Set SD communication pin
  pinMode(ChipSelect, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(Clock), rotate, LOW);

  Serial.begin(9600);
 
  // Initialize variables
  adjustValue = &currentColor.H;

  initSD();

  strip.Begin();
  strip.Show();
}

void loop() {
  long currentTime = millis();
  animations.UpdateAnimations();
  strip.Show();
  modeList[mode]();
  // Check if there are new params to be written and if we have waited sufficiently long between actions to write them
  if (newParamsSet > 0 && currentTime - newParamsSet >= 750) {
    String settingsStr = String("Mode: <") + mode + ">\n"
      "currentColor {Hue: <" + String(currentColor.H, 2) + "> | Saturation: <" + String(currentColor.S, 2) + "> | Brightness: <" + String(currentColor.B, 2) + ">}\n"
      "solidColor {Hue: <" + String(solidColor.H, 2) + "> | Saturation: <" + String(solidColor.S, 2) + "> | Brightness: <" + String(solidColor.B, 2) + ">}\n"
      "wipeColor {Hue: <" + String(wipeColor.H, 2) + "> | Saturation: <" + String(wipeColor.S, 2) + "> | Brightness: <" + String(wipeColor.B, 2) + ">}\n";
    char newSettings[256];
    settingsStr.toCharArray(newSettings, 256);
    writeParams(newSettings);
    newParamsSet = 0;
  }

  int b = checkButton();
  switch (b)
  {
    case 1:
      // advance to the next mode
      clickEvent();
      break;

    case 2:
      // cycle between coarse and fine adjustment
      doubleClickEvent();
      break;

    case 3:
      holdEvent();
      break;

    case 4:
      // reset all settings and return to standby mode
      longHoldEvent();
      break;

    default:
      break;
  }
}

//================================================================================
// Multiclick functions

int checkButton() {
  int event = 0;
  buttonVal = digitalRead(Switch);
  // Button pressed down
  if (buttonVal == LOW && buttonLast == HIGH && (millis() - upTime) > debounce)
  {
    downTime = millis();
    ignoreUp = false;
    waitForUp = false;
    singleOK = true;
    holdEventPast = false;
    longHoldEventPast = false;
    if ((millis() - upTime) < DCgap && DConUp == false && DCwaiting == true)
      DConUp = true;
    else
      DConUp = false;
    DCwaiting = false;
  }
  // Button released
  else if (buttonVal == HIGH && buttonLast == LOW && (millis() - downTime) > debounce)
  {
    if (not ignoreUp)
    {
      upTime = millis();
      if (DConUp == false) DCwaiting = true;
      else
      {
        event = 2;
        DConUp = false;
        DCwaiting = false;
        singleOK = false;
      }
    }
  }
  // Test for normal click event: DCgap expired
  if ( buttonVal == HIGH && (millis() - upTime) >= DCgap && DCwaiting == true && DConUp == false && singleOK == true && event != 2)
  {
    event = 1;
    DCwaiting = false;
  }
  // Test for hold
  if (buttonVal == LOW && (millis() - downTime) >= holdTime) {
    // Trigger "normal" hold
    if (not holdEventPast)
    {
      event = 3;
      waitForUp = true;
      ignoreUp = true;
      DConUp = false;
      DCwaiting = false;
      //downTime = millis();
      holdEventPast = true;
    }
    // Trigger "long" hold
    if ((millis() - downTime) >= longHoldTime)
    {
      if (not longHoldEventPast)
      {
        event = 4;
        longHoldEventPast = true;
      }
    }
  }
  buttonLast = buttonVal;
  return event;
}