#include <Arduino.h>
#include <FastLED.h>

#define MatrixHeight        8
#define MatrixLength        32
#define MAX_POWER_MILLIAMPS 1000
#define LED_TYPE            WS2812B
#define COLOR_ORDER         GRB
#define DATA_PIN            4
#define Brightness          255


#define NUM_LEDS MatrixHeight*MatrixLength

CRGB LedMatrix[NUM_LEDS];
CRGB LedTarget[NUM_LEDS];
uint8_t easetime[NUM_LEDS] = {100};

void genRandCol(CHSV& a, CHSV& b){
  byte hue_a = random(0,255);
  byte hue_b = (hue_a + 90 + random(0, 127))%360;
  byte sat_a = 178 + random(0, 255-178);
  byte sat_b = 178 + random(0, 255-178);

  a = CHSV(hue_a,sat_a,255);
  b = CHSV(hue_b,sat_b,255);

}

float Func(float x, float y, float t, uint8_t i) {

  float v;

  switch (i){
    case 1:
      v = sin( t - hypot( x-4, y-16 ) );
      break;
    case 2:
      v = sin(( t-x/2-y/2));
      break;
    case 3:
      v = sin((t-hypot(x-40.5,y-3.5))*0.25) ;
      break;
    case 4:
      v = sin(t-hypot(x-MatrixLength/2,y-3.5)) ;
      break;
    case 5:
      v = sin(t-x/2-y/2) ;
      break;
    case 6:
      v = sin(t+hypot(x-MatrixLength/2,y-MatrixHeight/2)) ;
      break;
    case 7:
      v = sin(t+x/2-y/2) ;
      break;
    default:
      v = cos(x+sin(t))-sin(y-cos(t)*0.5) ;
      break;
  }
   
 //float v = 2*fract( (0.5*t-x*0.01)*0.5+hypot(x-MatrixLength/2,y-MatrixHeight/2) )-1.0 ;
  return v ;//(v + 1.0 )/2;
}
CHSV interpolate_HSV_colors(CHSV& a, CHSV& b, fract8 value){
  return CHSV(
      lerp8by8(a.h, b.h, value),
      lerp8by8(a.s, b.s, value),
      lerp8by8(a.v, b.v, value)
    );
}
/* CRGB interpolate_colors_with_black(CRGB& a, CRGB& b, float value){
  CRGB X;

  if(value > 0){
    X.r = a.r * value; X.g = a.g * value; X.b = a.b * value;
  }else if (value < 0)
  {
    X.r = b.r * -value; X.g = b.g * -value; X.b = b.b * -value;
  }else{
    X.r = 0; X.g = 0; X.b = 0;
  }
  return X;
} */

CHSV interpolate_colors_with_black_rainbow(CHSV& a, CHSV& b, float value){

  if(value > 0){
    return CHSV(a.h * value, a.s, a.v);
  }else if (value < 0)
  {
    return CHSV(b.h * -value, b.s, b.v);
  }else{
    //return CHSV(0,0,0);
    return CHSV(( ( a.h + b.h ) / 2 ),  ( ( a.s + b.s ) / 2 ) , ( ( a.v + b.v ) / 2 ));
  }
}

CHSV interpolate_colors_with_black_s(CHSV& a, CHSV& b, float value){

  if(value > 0){
    return CHSV(a.h, a.s * value, a.v * value);
  }else if (value < 0)
  {
    return CHSV(b.h, b.s * -value, b.v * -value);
  }else{
    return CHSV( 0, 0 , 0 );
  }
}

CHSV interpolate_colors_with_sat(CHSV& a, CHSV& b, float value){

  if(value > 0){
    return CHSV(a.h, a.s * value, a.v * value);
  }else if (value < 0)
  {
    return CHSV(b.h, b.s * -value, b.v * -value);
  }else{
    return CHSV( 0, 0 , 0 );
  }
}

CHSV interpolate_colors_with_black_s_inv(CHSV& a, CHSV& b, float value){

  if(value > 0){
    return CHSV(a.h, 0xff-(a.s * value), a.v * value);
  }else if (value < 0)
  {
    return CHSV(b.h, 0xff-(b.s * -value), b.v * -value);
  }else{
    //return CHSV(0,0,0);
    return CHSV( ( ( a.h + b.h ) / 2 ) , ( ( a.s + b.s ) / 2 ) , 0xff );
  }
}

CHSV interpolate_colors_with_s_inv(CHSV& a, CHSV& b, float value){

  if(value > 0){
    return CHSV(a.h, 0xff-(a.s * value), a.v);
  }else if (value < 0)
  {
    return CHSV(b.h, 0xff-(b.s * -value), b.v);
  }else{
    //return CHSV(0,0,0);
    return CHSV( ( ( a.h + b.h ) / 2 ) , ( ( a.s + b.s ) / 2 )  , 0xff );
  }
}

CHSV interpolate_colors_with_black(CHSV& a, CHSV& b, float value){

  if(value > 0){
    return CHSV(a.h, a.s, a.v * value);
  }else if (value < 0)
  {
    return CHSV(b.h, b.s, b.v * -value);
  }else{
    return CHSV(0,0,0);
  }
}

struct Animation_States{
  float offset_x = 0;
  float offset_y = 0;
  float translate_scale = 11;
  float move_x = 1;
  float move_y = 1;
  float pivot_x = 0;
  float pivot_y = 0;
  float rotate_scale = 0.1;
  float zoom_scale = 3;
  int color_change_time = 0;
  int time_to_lerp_s = 5*1000;
  int lerp_time = 0;
  int speed = 100;
  CHSV A_old, B_old, A_new, B_new, A, B;
  uint8_t function = 3;
};


void lerp_toward_target_colors(int16_t dt, Animation_States& states){
  if(states.lerp_time <= states.time_to_lerp_s-dt){
    states.lerp_time += dt;
    fract8 v = ease8InOutQuad( ((float)states.lerp_time / (float)states.time_to_lerp_s) * 0xff );
    states.A = interpolate_HSV_colors(states.A_old, states.A_new, v);
    states.B = interpolate_HSV_colors(states.B_old, states.B_new, v);
  }else{
    states.A = states.A_new;
    states.B = states.B_new;
    states.A_old = states.A_new;
    states.B_old = states.B_new;
  }
}

void setPixel(float x, float y, CRGB pixel_color){
  int x_pos = x*MatrixHeight;
  int y_pos = y;
  if(((int)x & 0x1)) {
    y_pos = MatrixHeight-1 - y;
  }
  LedTarget[ x_pos+y_pos ] = pixel_color;
}

void ease(float dt){
  fract8 v = 127;
  for(uint16_t i = 0; i < NUM_LEDS; i++){
    //hsv2rgb_rainbow ( lerp_rgb(LedMatrix[i], LedTarget[i], v) , LedMatrix[i]);
    LedMatrix[i] = blend(LedMatrix[i], LedTarget[i], v);
  }
}

void Animation(float t, float dt, Animation_States& states){

  float val = 0;

  CHSV A = states.A;
  CHSV B = states.B;

  states.offset_x = dt * ( states.offset_x + states.move_x * 5 + states.move_x * states.speed ) + sin(0.3 + t * 0.17) * states.translate_scale;
  states.offset_y = dt * ( states.offset_y + states.move_y * 5 + states.move_y * states.speed ) + cos(0.7 + t * 0.05) * states.translate_scale;

  float pivot_x = states.pivot_x;
  float pivot_y = states.pivot_y;

  float zoom = 1.0;
  if(states.zoom_scale != 0){
    zoom = (sin(t * 0.1) * 0.5 + 0.5) * states.zoom_scale ;
  }

  float rotation = t * states.rotate_scale;

  for(uint16_t i = 0; i < NUM_LEDS; i++){
    uint16_t x = i/MatrixHeight;
    uint16_t y = i%MatrixHeight;
    float x_new = ( ( ( x - pivot_x  ) * cos(rotation) - ( y - pivot_y ) * sin(rotation) ) * zoom ) + pivot_x - states.offset_x;
    float y_new = ( ( ( y - pivot_y  ) * sin(rotation) + ( y - pivot_y  )* cos(rotation) ) * zoom ) + pivot_y - states.offset_y;
    //setPixel(x, y, interpolate_colors_with_black_rainbow(A, B, Func( x_new, y_new, t , states.function) ));
    //setPixel(x, y, interpolate_colors_with_black(A, B, Func( x_new, y_new, t , states.function) ));
    setPixel(x, y, interpolate_colors_with_s_inv(A, B, Func( x_new, y_new, t , states.function) ));
  } 
}

void adjust_gamma()
{
  for (uint16_t i = 0; i < NUM_LEDS; i++)
  {
    LedMatrix[i].r = 	dim8_lin (LedMatrix[i].r);
    LedMatrix[i].g = 	dim8_lin (LedMatrix[i].g);
    LedMatrix[i].b = 	dim8_lin (LedMatrix[i].b);
  }
}	

void setup() {
  FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(LedMatrix, NUM_LEDS).setCorrection( TypicalPixelString );
  FastLED.setMaxPowerInVoltsAndMilliamps( 5, MAX_POWER_MILLIAMPS);
  FastLED.setBrightness(Brightness);
  //Serial.begin(115200);
  randomSeed(analogRead(0));
}

void loop() {

  unsigned long NewTime, OldTime, dt_ms = 0;
  float seconds, Fps, dt = 0;
  Animation_States Ani_States;
  genRandCol(Ani_States.A, Ani_States.B);
  //Ani_States.function = random(0, 7);

  while(true){
    NewTime = millis();    
    dt_ms = (NewTime - OldTime);
    dt = dt_ms / 1000.0;
    OldTime = NewTime;
    seconds += dt;   

    if((int)seconds >= Ani_States.color_change_time){
      genRandCol(Ani_States.A_new, Ani_States.B_new);
      Ani_States.A_old = Ani_States.A;
      Ani_States.B_old = Ani_States.B;
      Ani_States.color_change_time = seconds + random(6, 10);
      Ani_States.lerp_time = 0;
      //Ani_States.function = random(0, 7);
    } 

    lerp_toward_target_colors(dt_ms, Ani_States);

    Animation(seconds, dt, Ani_States);
    //napplyGamma_video(LedMatrix, NUM_LEDS, 2.2);
    ease(dt);
    adjust_gamma();
    FastLED.show();
  }

}