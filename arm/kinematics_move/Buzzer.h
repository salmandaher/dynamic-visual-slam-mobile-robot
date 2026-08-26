#ifndef BUZZER_H
#define BUZZER_H

#include <Arduino.h>

void Buzzer_init();
void Buzzer_on();
void Buzzer_off();
void setBuzzer(int s);
void Buzzer_tone(int freq, int duration_ms);
void playMelody();
void beepOK();
void beepError();

#endif

