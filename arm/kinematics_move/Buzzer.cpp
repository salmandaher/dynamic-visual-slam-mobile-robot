#include "Buzzer.h"
#include "Arduino.h"

const int Buzzer_Pin = 27;
const int Buzzer_resolution = 10;

void Buzzer_init() {
    ledcAttach(Buzzer_Pin, 2000, Buzzer_resolution);
    ledcWrite(Buzzer_Pin, 0);
}

void Buzzer_on() {
    ledcWriteTone(Buzzer_Pin, 2000);
    ledcWrite(Buzzer_Pin, 512);
}

void Buzzer_off() {
    ledcWrite(Buzzer_Pin, 0);
}

void setBuzzer(int s) {
    Buzzer_on();
    delay(s);
    Buzzer_off();
}

void Buzzer_tone(int freq, int duration_ms) {
    if (freq <= 0) {
        ledcWrite(Buzzer_Pin, 0);
        delay(duration_ms);
        return;
    }

    ledcWriteTone(Buzzer_Pin, freq);
    ledcWrite(Buzzer_Pin, 512);
    delay(duration_ms);
    ledcWrite(Buzzer_Pin, 0);
    delay(20);
}

void beepOK() {
    Buzzer_tone(784, 120);
    Buzzer_tone(988, 120);
    Buzzer_tone(1175, 140);
    Buzzer_tone(1568, 260);
}

void beepError() {
    Buzzer_tone(330, 350);
    Buzzer_tone(220, 600);
}

void playMelody() {
    int notes[] = {
        523, 659, 784, 659,
        880, 784, 659, 523,
        659, 784, 988, 784,
        659, 523, 392, 0,
        523, 659, 784, 659,
        880, 784, 659, 523,
        494, 523, 659, 784,
        659, 523, 392, 0
    };

    int durations[] = {
        180, 180, 220, 180,
        220, 180, 180, 220,
        180, 180, 260, 180,
        180, 220, 320, 120,
        180, 180, 220, 180,
        220, 180, 180, 220,
        180, 180, 220, 260,
        220, 220, 400, 150
    };

    int count = sizeof(notes) / sizeof(notes[0]);

    for (int i = 0; i < count; i++) {
        Buzzer_tone(notes[i], durations[i]);
    }
}