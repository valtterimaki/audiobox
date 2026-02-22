#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

// ######## AUDIO ROUTINGS ########

AudioPlayWAVstereo       playSdWav1;
AudioPlayWAVstereo       playSdWav2;
AudioPlayWAVstereo       playSdWav3;
AudioPlayWAVstereo       playSdWav4;
AudioPlayWAVstereo       playSdWav5;
AudioFilterStateVariable filter1;        //xy=489,560
AudioMixer4              wavMixer2;         //xy=512,655
AudioMixer4              wavMixer;       //xy=696,497
AudioEffectFreeverb      reverb1;        //xy=1290,544
AudioMixer4              reverbMix;      //xy=1464,513
AudioAmplifier           reverbAmp;           //xy=1047,571
AudioAmplifier           amp1;           //xy=1629,507
AudioAnalyzeRMS          rms1;           //xy=1629,572
AudioOutputI2S           i2s1;           //xy=1781,504
AudioConnection          patchCord1(playSdWav4, 0, wavMixer2, 0);
AudioConnection          patchCord2(playSdWav5, 0, wavMixer2, 1);
AudioConnection          patchCord3(playSdWav3, 0, filter1, 0);
AudioConnection          patchCord4(playSdWav2, 0, wavMixer, 1);
AudioConnection          patchCord5(playSdWav1, 0, wavMixer, 0);
AudioConnection          patchCord6(filter1, 0, wavMixer, 2);
AudioConnection          patchCord7(wavMixer2, 0, wavMixer, 3);
AudioConnection          patchCord8(wavMixer, 0, reverbMix, 0);
AudioConnection          patchCord9(wavMixer, reverbAmp);
AudioConnection          patchCord10(reverbAmp, reverb1);
AudioConnection          patchCord11(reverb1, 0, reverbMix, 1);
AudioConnection          patchCord12(reverbMix, amp1);
AudioConnection          patchCord13(reverbMix, rms1);
AudioConnection          patchCord14(amp1, 0, i2s1, 0);

AudioControlSGTL5000     sgtl5000_1;

// ######## PINS ########

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


// ######## STRUCTS FOR TIMERS ########

struct Jump {
  uint8_t targetStep;
  uint8_t probability; // 0-100%
};

struct Step {
  uint32_t minTime;
  uint32_t maxTime;
  int minVal;
  int maxVal;
  uint8_t jumpCount;
  Jump jumps[3];
};

//######## GLOBAL VARIABLES ########

// Pins for the 8-way switch (TODO: reorder later)
const int switchPins[8] = {25, 26, 27, 28, 29, 30, 31, 32};

// Channel state trackers
int chan = 0;
int prev_chan = -1;
int pending_chan = -1;
bool is_transitioning = false;

// Compression
float compression;

// Blings controls
int blings_base = 1;
int blings_mult = 7;
int blings_notes[5];

// Track last trigger time for playSdWav1 through playSdWav5
uint32_t trackCooldowns[5] = {0, 0, 0, 0, 0};
const uint32_t GUARD_TIME = 200; // Minimum ms between re-triggers

//######## TIMER SEQUENCES ########

Step seqDefault[] =   { {500,  500,  0, 0, 0, {}} }; // Simple one second timer loop

Step seqPiano1[]      = { {10000, 11000, 0, 0, 0, {}} };          // A - Base piano
Step seqPiano2[]      = { {10000, 20000, 0, 0, 0, {}} };          // A - Additional piano 1
Step seqPiano3[]      = { {15000, 22000, 0, 0, 0, {}} };          // A - Additional piano 2

Step seqWails1[]      = { {11000, 13000, 0, 0, 0, {}} };          // C - Main echos
Step seqWails2[]      = { {15000, 22000, 0, 0, 0, {}} };          // C - Wails

Step seqRadio1[]      = { {1100,  1100,  1, 1, 0, {}},            // D - Radio mast words
                          {1100,  1100,  1, 1, 0, {}},
                          {1100,  1200,  1, 1, 3, {{5, 70}, {3, 30}, {4, 10}}},
                          {1100,  1100,  2, 2, 1, {{5, 100}}},
                          {1100,  1100,  3, 3, 0, {}}, 
                          {6000,  18000, 0, 0, 0, {}}};
Step seqRadio2[]      = { {10000, 30000, 0, 0, 0, {}} };          // D - Beep

Step seqChitter1[]    = { {9000,  17000, 0, 0, 0, {}} };          // E - Echos for the chitter

Step seqBlings1[]     = { {6000,  9000,  0, 0, 0, {}},            // B - Blings
                          {100,   400,   1, 1, 0, {}},
                          {100,   400,   2, 2, 0, {}},
                          {100,   400,   3, 3, 0, {}},
                          {100,   400,   4, 4, 0, {}}};
Step seqBlingsBase[]  = { {15000, 50000, 0, 0, 0, {}} };          // B - Base change

Step seqOpera1[]      = { {7500,  20000, 0, 0, 1, {{0, 70}}},     // J - Vocals
                          {7500,  9000,  0, 0, 1, {{1, 50}}} }; 

// ######## FUNCTIONS & CLASSES ########

class UniversalSequencer {
  private:
    Step* _steps;
    uint8_t _numSteps;
    uint8_t _currentIndex;
    uint32_t _lastMillis;
    uint32_t _currentWaitTime;
    bool _running;

    void prepareStep() {
      Step s = _steps[_currentIndex];
      _currentWaitTime = (s.minTime == s.maxTime) ? s.minTime : random(s.minTime, s.maxTime + 1);
    }

    uint8_t getNextIndex() {
      Step s = _steps[_currentIndex];
      if (s.jumpCount > 0) {
        int roll = random(0, 100);
        uint8_t cumulative = 0;
        for (uint8_t i = 0; i < s.jumpCount; i++) {
          cumulative += s.jumps[i].probability;
          if (roll < cumulative) return s.jumps[i].targetStep;
        }
      }
      return (_currentIndex + 1) % _numSteps;
    }

  public:
    UniversalSequencer(Step* steps, uint8_t count) : _steps(steps), _numSteps(count), _currentIndex(0), _running(false) {}

    void start(bool immediate = false) {
      _currentIndex = 0;
      _lastMillis = millis();
      _running = true;
      if (immediate) {
        _currentWaitTime = 0;
      } else {
        prepareStep();
      }
    }

    void stop() {
      _running = false;
    }

    void setSequence(Step* newSteps, uint8_t newCount) {
      _steps = newSteps;
      _numSteps = newCount;
      _currentIndex = 0; // Reset index for the new sequence
    }

    bool update(int &outputValue) {
      if (!_running) return false;
      
      uint32_t elapsed = millis() - _lastMillis;
      if (elapsed >= _currentWaitTime) {
        //Serial.printf("Timer Fired! Wait was: %d, Elapsed: %d\n", _currentWaitTime, elapsed);
        _lastMillis = millis();
        Step s = _steps[_currentIndex];
        outputValue = (s.minVal == s.maxVal) ? s.minVal : random(s.minVal, s.maxVal + 1);
        _currentIndex = getNextIndex();
        prepareStep();
        return true;
      }
      return false;
    }

    bool update() {
      int dummy;
      return update(dummy);
    }
};

// Inititate the sequencers

UniversalSequencer timerA(seqDefault, 1);
UniversalSequencer timerB(seqDefault, 1);
UniversalSequencer timerC(seqDefault, 1);
UniversalSequencer transitionTimer(seqDefault, 1);

// Random movement generator
class Drifter {
  private:
    float pos;
    float vel;
    float maxSpeed;
    float stepSize;
    unsigned long interval;     // Time between updates in milliseconds
    unsigned long lastUpdate;   // Timestamp of the last calculation

  public:
    // interval: time in ms between updates (e.g., 20 for smooth, 100 for slow)
    // speed: max velocity per update
    // smoothness: acceleration per update
    Drifter(float startPos, float speed, float smoothness, unsigned long updateInterval) {
      pos = startPos;
      maxSpeed = speed;
      stepSize = smoothness;
      interval = updateInterval;
      vel = 0;
      lastUpdate = 0;
    }

    void startAtRandom() {
      pos = random(0, 1001) / 1000.0; 
    }

    void startAt(float startvalue) {
      pos = startvalue;
    }

    float update() {
      unsigned long currentMillis = millis();

      // Only calculate new values if the interval has passed
      if (currentMillis - lastUpdate >= interval) {
        lastUpdate = currentMillis;

        // Apply Jitter (Acceleration)
        vel += ((float)random(1001) / 1000.0 - 0.5) * stepSize;
        vel = constrain(vel, -maxSpeed, maxSpeed);
        
        // Update Position
        pos += vel;
        
        // Debug
        //Serial.println(pos);

        // Reflective Boundaries
        if (pos <= 0) {
          pos = -pos;
          vel = -vel; 
        } else if (pos >= 1.0) {
          pos = 2.0 - pos;
          vel = -vel;
        }
      }

      return pos; // Returns the current position every time it's called
    }

    void setInterval(unsigned long ms) { interval = ms; }
    void setSpeed(float speed) { maxSpeed = speed; }
    void setSmoothness(float smoothness) { stepSize = smoothness; }
};

Drifter noiseLFO1(1, 0.01, 0.01, 50); 
Drifter noiseLFO2(1, 0.005, 0.01, 50); 
Drifter noiseLFO3(1, 0.007, 0.01, 50); 

void stopAll() {
  Serial.println("--- SYSTEM RESET (stopAll) ---");
  playSdWav1.stop();
  playSdWav2.stop();
  playSdWav3.stop();
  playSdWav4.stop();
  playSdWav5.stop();
  timerA.stop();
  timerB.stop();
  timerC.stop();

  for(int i=0; i<5; i++) trackCooldowns[i] = 0; // Reset guards
  delay(5);
}

void getUniqueRandoms(int* output, int numToPick, int maxRange) {
  // 1. Create a pool of all possible values (0 to maxRange-1)
  int pool[maxRange];
  for (int i = 0; i < maxRange; i++) {
    pool[i] = i;
  }

  // 2. Perform a partial shuffle for the number of values needed
  for (int i = 0; i < numToPick; i++) {
    // Pick a random index from the remaining pool
    int randomIndex = random(i, maxRange);

    // Swap the picked value with the current position i
    int temp = pool[i];
    pool[i] = pool[randomIndex];
    pool[randomIndex] = temp;

    // Store the result
    output[i] = pool[i];
  }
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

void playLoop(AudioPlayWAVstereo &track, int id, char bank, const char* trackname, int index) {
  // Ensure ID is within bounds (1-5)
  if (id < 1 || id > 5) return;
  int idx = id - 1;

  // Only proceed if the track is not playing AND the cooldown has passed
  if (!track.isPlaying() && (millis() - trackCooldowns[idx] > GUARD_TIME)) {
    playFile(track, bank, trackname, index);
    trackCooldowns[idx] = millis(); // Update the specific guard timer
  }
}

void handleChannelPlayback(int ch) {
  // 1. TRIGGER OR RE-TRIGGER TRANSITION
  // If the physical switch (ch) is different from our current target (pending_chan)
  if (ch != pending_chan) {
    wavMixer.gain(0, 1);
    wavMixer.gain(1, 1);
    wavMixer.gain(2, 1);
    wavMixer.gain(3, 1);
    wavMixer2.gain(0, 1);
    wavMixer2.gain(1, 1);
    reverbMix.gain(1, 0);
    reverbAmp.gain(0);
    reverb1.roomsize(0.0);
    stopAll();
    
    // Optional: Re-trigger transition sound
    //wavMixer.gain(0, 0.3);
    //playFile(playSdWav1, 'X', "1", random(1,15));
    playFile(playSdWav1, 'X', "N", 1); 
    
    pending_chan = ch;      // Update our destination to the newest switch position
    is_transitioning = true;
    transitionTimer.start(); // Restart the 1-second countdown
    
    Serial.printf("Switch moved to %d. (Re)starting transition.\n", pending_chan);
    return;
  }

  // 2. HANDLE THE TIMED DELAY
  if (is_transitioning) {
    if (transitionTimer.update()) {
      is_transitioning = false;
      playSdWav1.stop();
      wavMixer.gain(0, 1);
      reverbAmp.gain(1);
      prev_chan = pending_chan; // Confirm the target as the active channel
      setupChannelSpecifics(prev_chan);
      reverb1.roomsize(0.99);
      Serial.printf("Transition finalized. Active: %d\n", prev_chan);
    }
    return; // Block channel logic until timer expires
  }

  // 3. RUN ACTIVE CHANNEL LOGIC
  runActiveChannelLogic(prev_chan);
}

void setupChannelSpecifics(int ch) {

  switch (ch) {

    // THE PIANO
    case 0:
      //Set effects
      reverbMix.gain(1, 0);
      compression = 0;
      // Set timers
      timerA.setSequence(seqPiano1, 1);
      timerB.setSequence(seqPiano2, 1);
      timerC.setSequence(seqPiano2, 1);
      timerA.start(true);
      timerB.start();
      timerC.start();
      break;

    // THE WAILS
    case 1:
      // Set effects
      reverbMix.gain(1, 0);
      compression = 0;
      // Set timers
      timerA.setSequence(seqWails1, 1);
      timerB.setSequence(seqWails2, 1);
      timerA.start(true);
      timerB.start();
      break;

    // THE RADIO MAST
    case 2:
      // Set effects
      reverbMix.gain(1, 0);
      compression = 0;
      // Set timers
      timerA.setSequence(seqRadio1, 6);
      timerA.start();
      timerB.setSequence(seqRadio2, 1);
      timerB.start();
      break;

    // DISTANT CHITTER
    case 3:
      // Set effects
      reverbMix.gain(1, 0);
      compression = 0;
      // Set timers
      timerA.setSequence(seqChitter1, 1);
      timerA.start();
      break;

    // HUMMING
    case 4:
      // Set effects
      reverbMix.gain(1, 0);
      compression = 0;
      noiseLFO1.startAtRandom();
      break;

    // RUMBLE
    case 5:
      // Set effects
      reverbMix.gain(1, 0);
      compression = 0;
      break;

    // BLINGS
    case 6:
      // Set effects
      wavMixer.gain(0, 0.75);
      wavMixer.gain(1, 0.75);
      wavMixer.gain(2, 0.75);
      wavMixer.gain(3, 0.75);
      reverbMix.gain(1, 0.4);
      compression = 0;
      // Set timers
      timerA.setSequence(seqBlings1, 5);
      timerA.start(true);
      timerB.setSequence(seqBlingsBase, 1);
      timerB.start();
      break;

    case 7:
      // Set effects
      reverbMix.gain(1, 0.4);
      compression = 0;
      noiseLFO1.startAtRandom();
      noiseLFO3.startAt(0.5);
      wavMixer.gain(0, 0.7);
      wavMixer.gain(1, 0.7);
      // Set timers
      timerA.setSequence(seqOpera1, 2);
      timerA.start();

      break;

  }
}

void runActiveChannelLogic(int ch) {
  
  switch (ch) {

    // THE PIANO
    case 0:
    {

      // Piano rhythm, steady intervals
      if (timerA.update()) playFile(playSdWav1, 'A', "1", random(1,23));
      // Piano echos, slightly random intervals
      if (timerB.update()) playFile(playSdWav2, 'A', "2", random(1,24));
      if (timerC.update()) playFile(playSdWav3, 'A', "2", random(1,24));

      } break;

    // THE WAILS
    case 1:
    {
      // Main echos
      if (timerA.update()) playFile(playSdWav1, 'C', "1", random(1,105));
      // Wails
      if (timerB.update()) playFile(playSdWav2, 'C', "2", random(1,45));

      } break;

    // THE RADIO MAST
    case 2:
    {
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
      int result;
      if (timerA.update(result)) {
        if (result == 1) playFile(playSdWav1, 'D', "1", random(1,10));
        if (result == 2) playFile(playSdWav1, 'D', "1", random(13,20));
        if (result == 3) playFile(playSdWav1, 'D', "1", random(11,12));
      }
      // Noise
      playLoop(playSdWav2, 2, 'D', "3", 1);
      // Beep
      if (timerB.update()) playFile(playSdWav3, 'D', "4", 1);

      } break;

    // DISTANT CHITTER
    case 3:
    {
      // Echos
      if (timerA.update()) playFile(playSdWav1, 'E', "1", random(1,16));
      // Chitter
      playLoop(playSdWav2, 2, 'E', "2", 1);

      } break;

    // HUMMING
    case 4:
    {
      wavMixer.gain(0, noiseLFO1.update());
      wavMixer.gain(1, (1 - noiseLFO1.update()));
      // Loops
      playLoop(playSdWav1, 1, 'G', "1", 1);
      playLoop(playSdWav2, 2, 'G', "2", random(1,2));

      } break;

    // RUMBLE
    case 5:
    {
      // Changing rumble loops
      playLoop(playSdWav1, 1, 'H', "1", random(1, 3));
      // Pad
      playLoop(playSdWav2, 2, 'H', "bz", 1);
      // Noise
      playLoop(playSdWav3, 3, 'H', "ns", 1);

      } break;

    // BLINGS
    case 6: 
    {
      int result;
      if (timerA.update(result)) {
        if (result == 0) playFile(playSdWav1, 'B', "1", blings_base + (blings_mult * blings_notes[0]));
        if (result == 1) playFile(playSdWav2, 'B', "1", blings_base + (blings_mult * blings_notes[1]));
        if (result == 2) playFile(playSdWav3, 'B', "1", blings_base + (blings_mult * blings_notes[2]));
        if (result == 3) playFile(playSdWav4, 'B', "1", blings_base + (blings_mult * blings_notes[3]));
        if (result == 4) {
          playFile(playSdWav5, 'B', "1", blings_base + (blings_mult * blings_notes[4]));
          getUniqueRandoms(blings_notes, 5, 9);
        }
      }

      if (timerB.update()) blings_base = random(1,8);

      } break;

    // OPERA
    case 7: 
    {
      // Vocals
      float lfo1_snap = noiseLFO1.update();
      wavMixer.gain(0, lfo1_snap);
      wavMixer.gain(1, 1 - lfo1_snap);
      if (timerA.update()) {
        int opera_wav = random(1, 59);
        playFile(playSdWav1, 'J', "1", opera_wav);
        delay(10);
        playFile(playSdWav2, 'J', "2", opera_wav);
      } 

      // Chord
      float lfo3_snap = noiseLFO3.update();
      filter1.frequency((lfo3_snap*lfo3_snap*lfo3_snap)*1000.0 + 200.0);
      playLoop(playSdWav3, 3, 'J', "3", random(1,4));
      
    } break;
  
  }
}


// FOR DEBUG
unsigned long lastDebugPrint = 0;


// ######## SETUP ######## 

void setup() {
  Serial.begin(9600);

  randomSeed(analogRead(0)); // Seed the RNG

  timerA.start();
  timerB.start();
  timerC.start();

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
  sgtl5000_1.lineOutLevel(31);
  sgtl5000_1.audioPostProcessorEnable();
  sgtl5000_1.enhanceBassEnable();
  sgtl5000_1.enhanceBass(0.7, 1, 1, 1);

  // 8192 bytes is 8KB per track. Total 40KB for 5 tracks. 
  // The Teensy 4.1 has plenty of RAM, so we can afford this.
  playSdWav1.createBuffer(8192, AudioBuffer::inHeap);
  playSdWav2.createBuffer(8192, AudioBuffer::inHeap);
  playSdWav3.createBuffer(8192, AudioBuffer::inHeap);
  playSdWav4.createBuffer(8192, AudioBuffer::inHeap);
  playSdWav5.createBuffer(8192, AudioBuffer::inHeap);

  wavMixer.gain(0, 1);
  wavMixer.gain(1, 1);
  wavMixer.gain(2, 1);
  wavMixer.gain(3, 1);
  wavMixer2.gain(0, 1);
  wavMixer2.gain(1, 1);

  reverbMix.gain(0, 1);
  reverbMix.gain(1, 0);
  reverbAmp.gain(1);
  reverb1.roomsize(0.99);
  reverb1.damping(0.7);

  getUniqueRandoms(blings_notes, 5, 9);

  SPI.setMOSI(SDCARD_MOSI_PIN);
  SPI.setSCK(SDCARD_SCK_PIN);
  if (!(SD.begin(BUILTIN_SDCARD))) {
    while (1) {
      Serial.println("Unable to access the SD card");
      delay(500);
    }
  }
}


// ######## MAIN LOOP ########


void loop() {

  // Channel control (switch)
  for (int i = 0; i < 8; i++) {
    if (digitalRead(switchPins[i]) == 0) {
      chan = i;
    }
  }

  // Volume control
  // 1. Read the raw value (0 - 1023)
  float rawVal = analogRead(15);
  // 2. Normalize to 0.0 - 1.0
  float normalizedVol = rawVal / 1023.0;
  // 3. Apply the curve
  float curvedVol = normalizedVol * normalizedVol;
  // 4. Apply your max volume multiplier
  float finalGain = curvedVol;
  amp1.gain(finalGain * (1 - compression * 3));
  //Serial.println(vol);

  // Actual playback
  handleChannelPlayback(chan);

  
  /*if (millis() - lastDebugPrint > 1000) {
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
  }*/
  

}
