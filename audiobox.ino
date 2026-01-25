#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

// GUItool: begin automatically generated code
AudioPlaySdWav           playSdWav1;     //xy=664,523
AudioPlaySdWav           playSdWav2;     //xy=666,580
AudioMixer4              mixer1;         //xy=922,560
AudioOutputI2S           i2s1;           //xy=1182,560
AudioConnection          patchCord1(playSdWav1, 0, mixer1, 0);
AudioConnection          patchCord2(playSdWav2, 0, mixer1, 1);
AudioConnection          patchCord3(mixer1, 0, i2s1, 0);
AudioConnection          patchCord4(mixer1, 0, i2s1, 1);
// GUItool: end automatically generated code

AudioControlSGTL5000     sgtl5000_1;

// Use these with the Teensy Audio Shield
//#define SDCARD_CS_PIN    10
//#define SDCARD_MOSI_PIN  7   // Teensy 4 ignores this, uses pin 11
//#define SDCARD_SCK_PIN   14  // Teensy 4 ignores this, uses pin 13

// Use these with the Teensy 3.5 & 3.6 & 4.1 SD card
#define SDCARD_CS_PIN    BUILTIN_SDCARD
#define SDCARD_MOSI_PIN  11  // not actually used
#define SDCARD_SCK_PIN   13  // not actually used

// Use these for the SD+Wiz820 or other adaptors
//#define SDCARD_CS_PIN    4
//#define SDCARD_MOSI_PIN  11
//#define SDCARD_SCK_PIN   13


// ---------


// Pins for the 8-way switch (TODO: reorder later)
const int switchPins[8] = {25, 26, 27, 28, 29, 30, 31, 32}; 

// The wav files
// c + channel number, t + track number
const char *wav_c0_t0[8] = {"test_1.WAV", "test_2.WAV", "test_3.WAV", "test_4.WAV"};

// Channel state trackers
int chan = 0;
int prev_chan = 0;


// --- FUNCTIONS ---

void stopAll() {
  playSdWav1.stop();
  playSdWav2.stop();
  delay(5);
}

void playFile(AudioPlaySdWav &track, const char* file) {
  // Error failsafe
  if (!file || !file[0]) return;
  // Start playback, with error checking
  if (track.play(file)) {
    Serial.printf("Playing %s\n", file);
  } else {
    Serial.printf("ERROR: couldn't play %s\n", file);
  }
  delay(10); // wait for library to parse WAV info
}

void handleChannelPlayback(int ch) {

  switch (ch) {

    case 0:
      // Check first that the audio is not playing
      if (playSdWav1.isPlaying() == false) {  
        playFile(playSdWav1, wav_c0_t0[random(0,4)]);
      }
      break;

    case 1:
      break;

    case 2:
      break;

    case 3:
      break;

    case 4:
      break;

    case 5:
      break;

    case 6:
      break;

    case 7:
      break;
  }
}


// --- SETUP --- 


void setup() {
  Serial.begin(9600);

  for (int i = 0; i < 8; i++) {
    pinMode(switchPins[i], INPUT_PULLUP); // enables internal pull-up resistor
  }

  // Audio connections require memory to work.  For more
  // detailed information, see the MemoryAndCpuUsage example
  AudioMemory(80);

  // Comment these out if not using the audio adaptor board.
  // This may wait forever if the SDA & SCL pins lack
  // pullup resistors
  sgtl5000_1.enable();
  sgtl5000_1.volume(0.1);

  mixer1.gain(0, 0.2);
  mixer1.gain(1, 0.2);

  SPI.setMOSI(SDCARD_MOSI_PIN);
  SPI.setSCK(SDCARD_SCK_PIN);
  if (!(SD.begin(SDCARD_CS_PIN))) {
    // stop here, but print a message repetitively
    while (1) {
      Serial.println("Unable to access the SD card");
      delay(500);
    }
  }
}


// --- MAIN LOOP ---


void loop() {

  // Channel control (switch)
  for (int i = 0; i < 8; i++) {
    if (digitalRead(switchPins[i]) == 0) {
      chan = i;
    }
    //Serial.print(digitalRead(switchPins[i]));
    //Serial.print(", ");
  }
  //Serial.println("");
  if (prev_chan != chan) {
    stopAll();
    Serial.printf("Switching to channel %d\n", chan);
    prev_chan = chan;
  }

  // Volume control
  float vol = analogRead(15);
  vol = vol / 1024;
  //Serial.println(vol);
  mixer1.gain(0, vol);
  mixer1.gain(1, vol);


  // Actual playback
  handleChannelPlayback(chan);

}














