#include <Arduino.h>
#include <FastLED.h>

#define MatrixHeight        16
#define MatrixLength        16
#define MAX_POWER_MILLIAMPS 2000
#define LED_TYPE            WS2812B
#define COLOR_ORDER         GRB
#define DATA_PIN            4
#define Brightness          30


#define NUM_LEDS MatrixHeight*MatrixLength

CRGB LedMatrix[NUM_LEDS];

struct pixel{
  CRGB start_c = 0;
  CRGB target_c = 0;
  CRGB current_c = 0;
  int16_t easetime = 0;
  //unsigned long start_time = 0; 
  //uint8_t afterglow = 0;
};

pixel pMatrix[NUM_LEDS];

void setPixel(pixel& p, CRGB c){
  p.target_c = c;
  p.start_c = p.current_c;
	p.easetime = 0;
}

CRGB getcolor(pixel& p, float dt)
{
	if (p.easetime <= 0){
    p.easetime = 0;
		p.start_c = p.target_c;
		p.current_c = p.target_c;
		return p.current_c;
  }

	if (p.easetime > 100)
	{
		return p.current_c;
	}

	float ratio = ease8InOutCubic( float(p.easetime) / float(100) );

	p.current_c = blend(p.target_c, p.start_c, ratio);

  p.easetime -= (int)(dt*1000);

	return p.current_c;
}

void genRandCol(CRGB& a, CRGB& b){
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


CRGB interpolate_colors(CRGB& a, CRGB& b, float value){
  CRGB X;
  value = value*0xff;
  X.r = lerp8by8(a.r, b.r, value);
  X.g = lerp8by8(a.g, b.g, value);
  X.b = lerp8by8(a.b, b.b, value);
  return X;
}

CRGB interpolate_colors_with_black(CRGB& a, CRGB& b, float value){
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
  float time_to_lerp_s = 2;
  float lerp_time = 0;
  int speed = 100;
  CRGB A_old, B_old, A_new, B_new = 0;
  CRGB A, B = 0;
  uint8_t function = 3;
};

CHSV lerp_rgb(CRGB& old_c, CRGB& new_c, fract8 value){
  CHSV C;

  CHSV a = rgb2hsv_approximate (old_c);
  CHSV b = rgb2hsv_approximate (new_c);

  C.h = lerp8by8(a.h, b.h, value);
  C.s = lerp8by8(a.s, b.s, value);
  C.v = lerp8by8(a.v, b.v, value);

  return C;
}

CRGB lerp_toward_target_colors(CRGB& old, CRGB& target, float dt, Animation_States& states, float s){

  CRGB X = 0;

  if( ( (int)states.lerp_time <= (int)states.time_to_lerp_s ) && (target != old)){
    states.lerp_time += dt;
    //fract8 v = (int)( states.lerp_time / states.time_to_lerp_s )*0xff;
    int v = (int)( 0xff * s * 0.1) % 0xff;
    hsv2rgb_rainbow ( lerp_rgb(old, target, v) , X);   
  }else{
    states.lerp_time = 0;
    X = target;
    old = target;
  }
 

  return X;
}

void Animation(float t, float dt, Animation_States& states){

  float val = 0;

  CRGB A = states.A;
  CRGB B = states.B;

  states.offset_x = dt * ( states.offset_x + states.move_x * 5 + states.move_x * states.speed ) + sin(0.3 + t * 0.17) * states.translate_scale;
  states.offset_y = dt * ( states.offset_y + states.move_y * 5 + states.move_y * states.speed ) + cos(0.7 + t * 0.05) * states.translate_scale;

  float pivot_x = states.pivot_x;
  float pivot_y = states.pivot_y;

  float zoom = 1.0;
  if(states.zoom_scale != 0){
    zoom = (sin(t * 0.1) * 0.5 + 0.5) * states.zoom_scale ;
  }

  float rotation = t * states.rotate_scale;

  for(int x = 0; x < MatrixLength; ++x) {
    int x_pos = x*MatrixHeight;
    for(int y = 0; y < MatrixHeight; ++y) {

      int y_pos = y;
      if((x & 0x1)) {
        y_pos = MatrixHeight-1 - y;
      }

      float x_new = ( ( ( x - pivot_x  ) * cos(rotation) - ( y - pivot_y ) * sin(rotation) ) * zoom ) + pivot_x - states.offset_x;
      float y_new = ( ( ( y - pivot_y  ) * sin(rotation) + ( y - pivot_y  )* cos(rotation) ) * zoom ) + pivot_y - states.offset_y;

      LedMatrix[x_pos+y_pos] =  interpolate_colors_with_black(A, B, Func( x_new, y_new, t , states.function) );
      //setPixel(pMatrix[x_pos+y_pos], interpolate_colors_with_black(A, B, Func( x_new, y_new, t , states.function) ) );

    }
  }
}

void adjust_gamma()
{
  for (uint16_t i = 0; i < NUM_LEDS; i++)
  {
    LedMatrix[i].r = 	dim8_video (LedMatrix[i].r);
    LedMatrix[i].g = 	dim8_video (LedMatrix[i].g);
    LedMatrix[i].b = 	dim8_video (LedMatrix[i].b);
  }
}

void adjust_gamma2()
{
  for (uint16_t i = 0; i < NUM_LEDS; i++)
  {
    LedMatrix[i] = applyGamma_video(LedMatrix[i], 2.5);
   
  }
}
	

void setup() {
  FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(LedMatrix, NUM_LEDS).setCorrection( TypicalPixelString );
  FastLED.setMaxPowerInVoltsAndMilliamps( 5, MAX_POWER_MILLIAMPS);
  FastLED.setBrightness(Brightness);
  Serial.begin(115200);
  randomSeed(analogRead(0));
}

void display(float dt){
  for (uint16_t i = 0; i < NUM_LEDS; i++){
    LedMatrix[i] = getcolor(pMatrix[i], dt);
    //Serial.println(v.r);
  }
  FastLED.show();
}

void loop() {

  unsigned long NewTime, OldTime = 0;
  float seconds, Fps, dt = 0;
  Animation_States Ani_States;
  genRandCol(Ani_States.A, Ani_States.B);
  //Ani_States.function = random(0, 7);

  while(true){
    NewTime = millis();    
    dt = (NewTime - OldTime) / 1000.0;
    OldTime = NewTime;
    seconds += dt;   

    if((int)seconds >= Ani_States.color_change_time){
      /* genRandCol(Ani_States.A_new, Ani_States.B_new);
      Ani_States.A_old = Ani_States.A;
      Ani_States.B_old = Ani_States.B;*/
      Ani_States.color_change_time = seconds + random(10, 60); 
      //Ani_States.function = random(0, 7);
    } 
 

    //Ani_States.A = lerp_toward_target_colors(Ani_States.A_old, Ani_States.A_new, dt, Ani_States, seconds);
    //Ani_States.B = lerp_toward_target_colors(Ani_States.B_old, Ani_States.B_new, dt, Ani_States, seconds);

    Animation(seconds, dt, Ani_States);
    //napplyGamma_video(LedMatrix, NUM_LEDS, 2.2);
    adjust_gamma();
    //blur1d(LedMatrix , 256 , 32 );
    //display(dt);
    FastLED.show();
  }

}