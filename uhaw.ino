#define led 23 // led pin
#define buzzerpin 19 // buzzer pin

int noteIndex = -1;           
int noteDuration = 0;    
unsigned long currentT;
unsigned long previousT = 0;


#define C6 1047
#define C_6 1109
#define D6 1175
#define D_6 1245
#define E6 1319
#define F6 1397
#define F_6 1480
#define G6 1568
#define G_6 1661
#define A6 1760
#define A_6 1865
#define B6 1976
#define C7 2093
#define C_7 2217
#define D7 2349
#define D_7 2489
#define E7 2637
#define F7 2794
#define F_7 2960
#define G7 3136
#define G_7 3322
#define A7 3520
#define A_7 3729
#define B7 3951
#define C8 4186
#define C_8 4435
#define D8 4699
#define D_8 4978
#define E8 5274
#define F8 5588
#define F_8 5920
#define G8 6272
#define G_8 6645
#define A8 7040
#define A_8 7458
#define B8 7902
#define C9 8372
#define C_9 8869
#define D9 9397
#define D_9 9956
#define E9 10548
#define F9 11175
#define F_9 11840
#define G9 12544
#define G_9 13290
#define A9 14080
#define A_9 14917
#define B9 15804
#define C10 16744

int melody[] = {
    G7, G7, G7, E8, E8, G8, E8, D8,  // Triple notes
    D8, E8, D8, C8, A7,              // Rest and whole notes
    G7, A7, G7, E8, E8,              // Notes
    E8, G8, E8, D8,                  // 2 notes
    D8, E8, C8, A7, A7,              // 3 notes
    0, G7, G7, G7, G7, G7, G8,       // G7 repeated with G8
    E8, E8, E8, G8, E8, G8, E8,      // Notes
    E8, G8, E8, A8,                  // Notes
    G8, G8, E8, E8, D8,              // Notes
    0, G7, G7, G7, G7, G7, G8,       // G7 repeated with G8
    E8, E8, E8, D8,                  // 3 notes
    C8, A7                           // Notes
};
int noteDurations[] = {
     8, 8, 8, 2,             // Section 1
    4, 4, 4, 2,             // Section 2
    8, 8, 8, 8, 1,          // Section 3
    8, 8, 8, 8, 2,          // Section 4

    4, 4, 4, 2,             // Section 5
    8, 8, 8, 8, 1,          // Section 6
    8, 8, 8, 8, 8, 8, 8, 8, // Section 7
    2, 8, 4, 8, 4, 4,       // Section 8

    8, 8, 8, 2,             // Section 9
    8, 8, 8, 8, 1,          // Section 10
    8, 8, 8, 8, 8, 4, 8, 2, // Section 11
    4, 2, 4, 1,             // Section 12
};

void setup() {
  pinMode(buzzerpin,OUTPUT);
  pinMode(led,OUTPUT);
}

void loop() {
  buzzer_sound();

}


void buzzer_sound(){
  currentT = millis();
  digitalWrite(led,LOW);
    if (currentT - previousT >= noteDuration * .5) {
    digitalWrite(led,HIGH);
    noTone(buzzerpin);
    noteIndex++;
    if (noteIndex >= sizeof(melody) / sizeof(melody[0])) {
      noteIndex = 0;
    }
    noteDuration = 4000 / noteDurations[noteIndex];
    tone(buzzerpin, melody[noteIndex], noteDuration);
    previousT = currentT;
  }
}

