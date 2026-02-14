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
AudioMixer4              wavMixer;         //xy=388,594
AudioEffectDelay         delay1;         //xy=587,673
AudioMixer4              echoMixer;         //xy=794,591
AudioEffectFreeverb      reverb1;      //xy=982,641
AudioMixer4              reverbMix;         //xy=1156,610
AudioAmplifier           amp1;           //xy=1321,604
AudioAnalyzeRMS          rms1;           //xy=1321,669
AudioOutputI2S           i2s1;           //xy=1473,601
AudioConnection          patchCord1(playSdWav1, 0, wavMixer, 0);
AudioConnection          patchCord2(playSdWav2, 0, wavMixer, 1);
AudioConnection          patchCord3(playSdWav3, 0, wavMixer, 2);
AudioConnection          patchCord4(playSdWav4, 0, wavMixer, 3);
AudioConnection          patchCord5(wavMixer, delay1);
AudioConnection          patchCord6(wavMixer, 0, echoMixer, 0);
AudioConnection          patchCord7(delay1, 0, echoMixer, 1);
AudioConnection          patchCord8(delay1, 1, echoMixer, 2);
AudioConnection          patchCord9(delay1, 2, echoMixer, 3);
AudioConnection          patchCord10(echoMixer, 0, reverbMix, 0);
AudioConnection          patchCord11(echoMixer, reverb1);
AudioConnection          patchCord12(reverb1, 0, reverbMix, 1);
AudioConnection          patchCord13(reverbMix, amp1);
AudioConnection          patchCord14(reverbMix, rms1);
AudioConnection          patchCord15(amp1, 0, i2s1, 0);
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
int prev_chan = -1;

// word status
int word_status;

// Compression
float compression;


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

SimpleTimer timerA(1000);
SimpleTimer timerB(1000);
SimpleTimer timerC(1000);

class RadioMastTimer {
  private:
    uint32_t lastMillis;
    uint32_t interval;
    uint32_t nextStep;
    bool enabled;

  public:
    RadioMastTimer(uint32_t intervalMillis = 1000) {
      interval = intervalMillis;
      lastMillis = 0;
      enabled = false;
      // Use this to determine next step. 0, 1 & 2 are for the numbers, 3 is for cardinals, 4 is the levels.
      nextStep = 0; 
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
    int isReady() {
      if (!enabled) return -1;
      
      // Is interval triggered?
      if (millis() - lastMillis >= interval) {

        if (nextStep == 0) { // Is the next word the numbers
          lastMillis = millis(); 
          nextStep = 1;
          setInterval(1100);
          return 0;
          
        } else if (nextStep == 1) { // Is the next word the cardinals
          lastMillis = millis(); 
          nextStep = 2;
          setInterval(1100);
          return 0;

        } else if (nextStep == 2) { // Is the next word the cardinals
          lastMillis = millis(); 
          int chance = random(100);
          if (chance <= 50){
            nextStep = 0;
            setInterval(random(4000, 6000));
          } else if (chance > 50 && chance < 90) {
            nextStep = 3;
            setInterval(2000);
          } else {
            nextStep = 4;
            setInterval(2000);
          }
          return 0;

        } else if (nextStep == 3) { // Is the next word the cardinals
          lastMillis = millis(); 
          nextStep = 0;
          setInterval(random(4000, 6000));
          return 1;

        } else if (nextStep == 4) { // Is the next word the levels
          lastMillis = millis();
          nextStep = 0;
          setInterval(random(4000, 6000));
          return 2;

        }
      }
      return -1;
    }
};

RadioMastTimer mast_timer(1000);

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

    // THE PIANO
    case 0:
      // Initial start operations
      if (prev_chan != chan) {
        //Set effects
        echoMixer.gain(1, 0);
        echoMixer.gain(2, 0);
        echoMixer.gain(3, 0);
        reverbMix.gain(1, 0);
        // Set timers
        timerA.setInterval(10000);
        timerB.setInterval(random(10000, 20000));
        timerC.setInterval(random(15000, 22000));
        timerA.start();
        timerB.start();
        timerC.start();
        // Stop all and change prev chan tracker value
        stopAll();
        Serial.printf("Switching to channel %d\n", chan);
        prev_chan = chan;
      }
    
      // Piano rhythm, steady intervals
      if (timerA.isReady()) { 
        playFile(playSdWav1, 'A', "1", random(1,23));
      }
      // Piano echos, slightly random intervals
      if (timerB.isReady()) { 
        playFile(playSdWav2, 'A', "2", random(1,24));
        timerB.setInterval(random(10000, 20000));
      }
      if (timerC.isReady()) { 
        playFile(playSdWav3, 'A', "2", random(1,24));
        timerC.setInterval(random(15000, 22000));
      }

      break;

    // THE WAILS
    case 1:

      // Initial start operations
      if (prev_chan != chan) {
        // Set effects
        echoMixer.gain(1, 0);
        echoMixer.gain(2, 0);
        echoMixer.gain(3, 0);
        reverbMix.gain(1, 0);
        // Set timers
        timerA.setInterval(random(11000, 13000));
        timerB.setInterval(random(15000, 22000));
        timerA.start();
        timerB.start();
        // Stop all and change prev chan tracker value
        stopAll();
        Serial.printf("Switching to channel %d\n", chan);
        prev_chan = chan;
      }
    
      // Main echos
      if (timerA.isReady()) { 
        playFile(playSdWav1, 'C', "1", random(1,105));
        timerA.setInterval(random(12000, 14000));
      }
      // Wails
      if (timerB.isReady()) { 
        playFile(playSdWav2, 'C', "2", random(1,45));
        timerB.setInterval(random(14000, 20000));
      }

      break;

    // THE RADIO MAST
    case 2:

      // Initial start operations
      if (prev_chan != chan) {
        // Set effects
        delay1.delay(0, 100);
        delay1.delay(1, 200);
        delay1.delay(2, 300);
        echoMixer.gain(1, 0.1);
        echoMixer.gain(2, 0.002);
        echoMixer.gain(3, 0.001);
        reverbMix.gain(1, 0);
        compression = 0;
        // Set timers
        timerA.setInterval(2000);
        timerB.setInterval(2000);
        timerA.start();
        timerB.start();
        mast_timer.start();
        // Stop all and change prev chan tracker value
        stopAll();
        Serial.printf("Switching to channel %d\n", chan);
        prev_chan = chan;
      }

      
      if (rms1.available()) {
        float last_rms = rms1.read();
        //float comp_mult = 5.0;
        if (last_rms > compression) compression = (compression + last_rms) / 2;
        if (last_rms <= compression) compression -= 0.001;
        /*if (millis() % 10 == 1) {
          Serial.print(last_rms);
          Serial.print(" + ");
          Serial.println(compression);
        }*/
      }
   
    
      // Words
      word_status = mast_timer.isReady(); 
      if (word_status == 0) playFile(playSdWav1, 'D', "1", random(1,10));
      if (word_status == 1) playFile(playSdWav1, 'D', "1", random(13,20));
      if (word_status == 2) playFile(playSdWav1, 'D', "1", random(11,12));

      if (timerA.isReady()) playFile(playSdWav2, 'D', "3", 1);

      break;

    case 3:

      // Initial start operations
      if (prev_chan != chan) {
        // Set effects
        // Set timers
        // Stop all and change prev chan tracker value
        stopAll();
        Serial.printf("Switching to channel %d\n", chan);
        prev_chan = chan;
      }
      break;

    case 4:

      // Initial start operations
      if (prev_chan != chan) {
        // Set effects
        // Set timers
        // Stop all and change prev chan tracker value
        stopAll();
        Serial.printf("Switching to channel %d\n", chan);
        prev_chan = chan;
      }

      break;

    case 5:

      // Initial start operations
      if (prev_chan != chan) {
        // Set effects
        // Set timers
        // Stop all and change prev chan tracker value
        stopAll();
        Serial.printf("Switching to channel %d\n", chan);
        prev_chan = chan;
      }
      break;

    case 6:

      // Initial start operations
      if (prev_chan != chan) {
        // Set effects
        // Set timers
        // Stop all and change prev chan tracker value
        stopAll();
        Serial.printf("Switching to channel %d\n", chan);
        prev_chan = chan;
      }
      break;

    case 7:

      // Initial start operations
      if (prev_chan != chan) {
        // Set effects
        // Set timers
        // Stop all and change prev chan tracker value
        stopAll();
        Serial.printf("Switching to channel %d\n", chan);
        prev_chan = chan;
      }
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

  for (int i = 0; i < 8; i++) {
    pinMode(switchPins[i], INPUT_PULLUP); // enables internal pull-up resistor
  }

  // Audio connections require memory to work.  For more
  // detailed information, see the MemoryAndCpuUsage example
  AudioMemory(1024);

  // Comment these out if not using the audio adaptor board.
  // This may wait forever if the SDA & SCL pins lack
  // pullup resistors
  sgtl5000_1.enable();
  sgtl5000_1.volume(0.1);

  playSdWav1.createBuffer(2048,AudioBuffer::inHeap);
  playSdWav2.createBuffer(2048,AudioBuffer::inHeap);
  playSdWav3.createBuffer(2048,AudioBuffer::inHeap);
  playSdWav4.createBuffer(2048,AudioBuffer::inHeap);

  wavMixer.gain(0, 1);
  wavMixer.gain(1, 1);
  wavMixer.gain(2, 1);
  wavMixer.gain(3, 1);

  delay1.delay(0, 800);
  delay1.delay(1, 1600);
  delay1.delay(2, 2400);

  echoMixer.gain(0, 1);
  echoMixer.gain(1, 0.5);
  echoMixer.gain(2, 0.25);
  echoMixer.gain(3, 0.125);

  reverbMix.gain(0, 1);
  reverbMix.gain(1, 0.5);
  reverb1.roomsize(0.99);
  reverb1.damping(0.4);

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
  }

  // Volume control
  float vol = analogRead(15);
  vol = vol / 1024;
  //Serial.println(vol);
  amp1.gain(vol * (1 - compression*3));

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
