#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

// GUItool: begin automatically generated code
AudioPlayWAVstereo       playSdWav1;
AudioPlayWAVstereo       playSdWav2;
AudioPlayWAVstereo       playSdWav3;
AudioPlayWAVstereo       playSdWav4;
AudioEffectFreeverb      reverb1;      
AudioMixer4              mixer1;
AudioOutputI2S           i2s1;
AudioConnection          patchCord1(playSdWav1, 0, mixer1, 0);
AudioConnection          patchCord2(playSdWav2, 0, mixer1, 1);
AudioConnection          patchCord3(playSdWav3, 0, mixer1, 2);
AudioConnection          patchCord4(playSdWav4, 0, mixer1, 3);
AudioConnection          patchCord5(mixer1, reverb1);
AudioConnection          patchCord6(reverb1, 0, i2s1, 0);
AudioConnection          patchCord7(reverb1, 0, i2s1, 1);
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

// Channel state trackers
int chan = 0;
int prev_chan = 0;


// --- FUNCTIONS & CLASSES ---

class SimpleTimer {
  private:
    uint32_t lastMillis;
    uint32_t interval;
    bool enabled;

  public:
    SimpleTimer(uint32_t intervalMillis = 1000) {
      interval = intervalMillis;
      lastMillis = 0;
      enabled = false;
    }

    void start() {
      lastMillis = millis();
      enabled = true;
    }

    void stop() {
      enabled = false;
    }

    void setInterval(uint32_t newInterval) {
      interval = newInterval;
    }

    // This is the "polling" function
    bool isReady() {
      if (!enabled) return false;
      
      if (millis() - lastMillis >= interval) {
        lastMillis = millis(); // Reset for the next interval
        return true;
      }
      return false;
    }
};

SimpleTimer timerA(10000);
SimpleTimer timerB(10000);
SimpleTimer timerC(10000);


void stopAll() {
  playSdWav1.stop();
  playSdWav2.stop();
  playSdWav3.stop();
  playSdWav4.stop();
  delay(5);
}

void playFile(AudioPlayWAVstereo &track, char bank, const char* trackname, int index) {

  // 1. Calculate buffer size: 1 (char) + strlen(descriptor) + ~3 (digits) + 5 (.WAV) + 1 (null)
  // 32 bytes is a safe, conservative buffer for most Arduino filenames.
  char filenameBuffer[32]; 

  // 2. Format: "A_tracknumber_1.WAV"
  // %c = char, %s = string, %d = integer
  sprintf(filenameBuffer, "%c_%s_%d.WAV", bank, trackname, index);

  // Error failsafe
  if (filenameBuffer[0] == '\0') return; 
  // Start playback, with error checking
  if (track.play(filenameBuffer)) {
    Serial.printf("Playing %s\n", filenameBuffer);
  } else {
    Serial.printf("ERROR: couldn't play %s\n", filenameBuffer);
  }

}

void handleChannelPlayback(int ch) {

  switch (ch) {

    case 0:
      // Piano rhythm, steady intervals
      if (timerA.isReady()) { 
        playFile(playSdWav1, 'A', "1", random(1,23));
      }
      // Piano echos, slightly random intervals
      if (timerB.isReady()) { 
        playFile(playSdWav2, 'A', "2", random(1,24));
        timerB.setInterval(random(10000, 20000));
      }
      if (timerB.isReady()) { 
        playFile(playSdWav3, 'A', "2", random(1,24));
        timerB.setInterval(random(15000, 22000));
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

// DEBUG
unsigned long lastDebugPrint = 0;


// --- SETUP --- 

void setup() {
  Serial.begin(9600);

  timerA.start();
  timerB.start();
  timerC.start();

  reverb1.roomsize(0.99);
  reverb1.damping(0.01);

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

  playSdWav1.createBuffer(2048,AudioBuffer::inHeap);
  playSdWav2.createBuffer(2048,AudioBuffer::inHeap);
  playSdWav3.createBuffer(2048,AudioBuffer::inHeap);
  playSdWav4.createBuffer(2048,AudioBuffer::inHeap);

  mixer1.gain(0, 0.5);
  mixer1.gain(1, 0.5);

  SPI.setMOSI(SDCARD_MOSI_PIN);
  SPI.setSCK(SDCARD_SCK_PIN);
  if (!(SD.begin(BUILTIN_SDCARD))) {
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

  
  if (millis() - lastDebugPrint > 1000) {
    Serial.print("Current Mem: ");
    Serial.println(AudioMemoryUsage());
    Serial.print("Memory: ");
    Serial.print(AudioMemoryUsageMax());
    Serial.print(" | CPU: ");
    Serial.print(AudioProcessorUsage());
    Serial.println("%");
    
    // Reset the max counter so we see current spikes, not old ones
    AudioMemoryUsageMaxReset(); 
    lastDebugPrint = millis();
    }
  

}
