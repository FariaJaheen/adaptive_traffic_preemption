/*
  Adaptive Traffic Lights with Emergency Vehicle Preemption (Two-Button Version)
  Compile-safe for Arduino IDE auto-prototyping (no custom types in prototypes)

  Signals (LEDs):
    NS: D2(G), D3(Y), D4(R)
    EW: D5(G), D6(Y), D7(R)

  Detectors (LDR voltage dividers):
    NS -> A0  (occupied when A0 < TH_NS)
    EW -> A1

  Pedestrian buttons (INPUT_PULLUP):
    PED_NS -> D8  (cross EW roadway)
    PED_EW -> D9  (cross NS roadway)

  Emergency preemption buttons (INPUT_PULLUP, separate):
    PREEMPT_NS -> D10 (priority for NS)
    PREEMPT_EW -> D11 (priority for EW)

  Telemetry: 115200 baud
*/

// ---------- Timings (ms) ----------
const unsigned long T_MIN_GREEN   = 7000;
const unsigned long T_MAX_GREEN   = 22000;
const unsigned long T_EXT_STEP    = 1500;
const unsigned long T_YELLOW      = 2500;
const unsigned long T_ALL_RED     = 900;

const unsigned long T_PED_WALK    = 5000;
const unsigned long T_PED_FDW     = 7000;   // Flashing Don't Walk window
const unsigned long T_PED_FLASH   = 250;    // 2 Hz flash

const unsigned long T_PREEMPT_GREEN = 10000; // guaranteed green during preempt
const unsigned long T_PREEMPT_HOLD  = 4000;  // all-red recovery after preempt

// ---------- Pins ----------
const int NS_G = 2, NS_Y = 3, NS_R = 4;
const int EW_G = 5, EW_Y = 6, EW_R = 7;

const int BTN_PED_NS = 8;
const int BTN_PED_EW = 9;

const int BTN_PREEMPT_NS = 10;
const int BTN_PREEMPT_EW = 11;

const int DET_NS = A0;
const int DET_EW = A1;

// ---------- Detector thresholds & hysteresis ----------
int TH_NS = 500;  // <500 => vehicle present (tune)
int TH_EW = 500;

const int HYST_SAMPLES = 3;
int occNS_count = 0, occEW_count = 0;

bool occNS() {
  int v = analogRead(DET_NS);
  if (v < TH_NS) { if (occNS_count < HYST_SAMPLES) occNS_count++; }
  else           { if (occNS_count > 0) occNS_count--; }
  return occNS_count >= HYST_SAMPLES;
}
bool occEW() {
  int v = analogRead(DET_EW);
  if (v < TH_EW) { if (occEW_count < HYST_SAMPLES) occEW_count++; }
  else           { if (occEW_count > 0) occEW_count--; }
  return occEW_count >= HYST_SAMPLES;
}

// ---------- Button debounce ----------
struct DebBtn { int pin; bool last; unsigned long t; };
DebBtn bPedNS{BTN_PED_NS, true, 0};
DebBtn bPedEW{BTN_PED_EW, true, 0};
DebBtn bPreNS{BTN_PREEMPT_NS, true, 0};
DebBtn bPreEW{BTN_PREEMPT_EW, true, 0};

bool falling(DebBtn& b, unsigned long now, unsigned long db=30){
  bool s = digitalRead(b.pin); // INPUT_PULLUP: LOW=pressed
  if (s != b.last && (now - b.t) > db) { b.last = s; b.t = now; return (s==LOW); }
  return false;
}

// ---------- Phases as integers (no custom enum type) ----------
const uint8_t NS_G_PHASE = 0;
const uint8_t NS_Y_PHASE = 1;
const uint8_t EW_G_PHASE = 2;
const uint8_t EW_Y_PHASE = 3;
const uint8_t ALL_RED_PHASE = 4;
const uint8_t PED_NS_WALK = 5;
const uint8_t PED_NS_FDW  = 6;
const uint8_t PED_EW_WALK = 7;
const uint8_t PED_EW_FDW  = 8;
const uint8_t PREEMPT_NS_PHASE = 9;
const uint8_t PREEMPT_EW_PHASE = 10;
const uint8_t PREEMPT_HOLD_PHASE = 11;

// Forward prototypes to satisfy Arduino preprocessor
void enterPhase(uint8_t next);
bool timeUp(unsigned long dur);
void allRed();
void setLamps(bool nsG,bool nsY,bool nsR,bool ewG,bool ewY,bool ewR);

// ---------- Global state ----------
uint8_t phase = NS_G_PHASE;
unsigned long t0 = 0;              // phase start time
unsigned long greenBudget = 0;     // adaptive green budget (min + extensions)
bool servedNSLast = true;          // which vehicle phase was last
int maxHitsNS = 0, maxHitsEW = 0;  // starvation guard counters
bool pedReqNS = false, pedReqEW = false;

const uint8_t PRE_NONE = 0, PRE_NS = 1, PRE_EW = 2;
uint8_t pendingPreempt = PRE_NONE;

const uint8_t APP_NONE = 0, APP_NS = 1, APP_EW = 2;
uint8_t lastPreempt = APP_NONE;

// ---------- Utilities ----------
void setLamps(bool nsG,bool nsY,bool nsR,bool ewG,bool ewY,bool ewR){
  digitalWrite(NS_G, nsG); digitalWrite(NS_Y, nsY); digitalWrite(NS_R, nsR);
  digitalWrite(EW_G, ewG); digitalWrite(EW_Y, ewY); digitalWrite(EW_R, ewR);
}
void allRed(){ setLamps(0,0,1, 0,0,1); }

void enterPhase(uint8_t next){
  phase = next; t0 = millis();
  switch(phase){
    case NS_G_PHASE: setLamps(1,0,0, 0,0,1); greenBudget = T_MIN_GREEN; break;
    case NS_Y_PHASE: setLamps(0,1,0, 0,0,1); break;
    case EW_G_PHASE: setLamps(0,0,1, 1,0,0); greenBudget = T_MIN_GREEN; break;
    case EW_Y_PHASE: setLamps(0,0,1, 0,1,0); break;
    case ALL_RED_PHASE: allRed(); break;
    case PED_NS_WALK:  allRed(); break;
    case PED_NS_FDW:   allRed(); break;
    case PED_EW_WALK:  allRed(); break;
    case PED_EW_FDW:   allRed(); break;
    case PREEMPT_NS_PHASE: setLamps(1,0,0, 0,0,1); lastPreempt = APP_NS; break;
    case PREEMPT_EW_PHASE: setLamps(0,0,1, 1,0,0); lastPreempt = APP_EW; break;
    case PREEMPT_HOLD_PHASE: allRed(); break;
  }

  // Telemetry
  Serial.print("PHASE=");
  switch(phase){
    case NS_G_PHASE: Serial.print("NS_G"); break;
    case NS_Y_PHASE: Serial.print("NS_Y"); break;
    case EW_G_PHASE: Serial.print("EW_G"); break;
    case EW_Y_PHASE: Serial.print("EW_Y"); break;
    case ALL_RED_PHASE: Serial.print("ALL_RED"); break;
    case PED_NS_WALK: Serial.print("PED_NS_WALK"); break;
    case PED_NS_FDW:  Serial.print("PED_NS_FDW"); break;
    case PED_EW_WALK: Serial.print("PED_EW_WALK"); break;
    case PED_EW_FDW:  Serial.print("PED_EW_FDW"); break;
    case PREEMPT_NS_PHASE: Serial.print("PREEMPT_NS"); break;
    case PREEMPT_EW_PHASE: Serial.print("PREEMPT_EW"); break;
    case PREEMPT_HOLD_PHASE: Serial.print("PREEMPT_HOLD"); break;
  }
  Serial.print(" t="); Serial.println(t0);
}

bool timeUp(unsigned long dur){ return millis() - t0 >= dur; }

// ---------- Setup ----------
void setup(){
  pinMode(NS_G,OUTPUT); pinMode(NS_Y,OUTPUT); pinMode(NS_R,OUTPUT);
  pinMode(EW_G,OUTPUT); pinMode(EW_Y,OUTPUT); pinMode(EW_R,OUTPUT);

  pinMode(BTN_PED_NS, INPUT_PULLUP);
  pinMode(BTN_PED_EW, INPUT_PULLUP);
  pinMode(BTN_PREEMPT_NS, INPUT_PULLUP);
  pinMode(BTN_PREEMPT_EW, INPUT_PULLUP);

  Serial.begin(115200);
  enterPhase(NS_G_PHASE);
}

// ---------- Inputs ----------
void handlePedButtons(unsigned long now){
  if (falling(bPedNS, now)) pedReqNS = true; // crossing EW roadway
  if (falling(bPedEW, now)) pedReqEW = true; // crossing NS roadway
}
void handlePreemptButtons(unsigned long now){
  if (falling(bPreNS, now)) pendingPreempt = PRE_NS;
  if (falling(bPreEW, now)) pendingPreempt = PRE_EW;
}

// ---------- Ped helpers ----------
void servePedNSWalkFDW(){ enterPhase(PED_NS_WALK); }
void servePedEWWalkFDW(){ enterPhase(PED_EW_WALK); }

// ---------- Main loop ----------
void loop(){
  unsigned long now = millis();

  handlePedButtons(now);
  handlePreemptButtons(now);

  // Guide to safe preempt sequence if a request is pending
  if (pendingPreempt != PRE_NONE) {
    switch(phase){
      case NS_G_PHASE:
        if (pendingPreempt == PRE_NS) { enterPhase(PREEMPT_NS_PHASE); }
        else { if (timeUp(T_MIN_GREEN)) enterPhase(NS_Y_PHASE); }
        break;
      case EW_G_PHASE:
        if (pendingPreempt == PRE_EW) { enterPhase(PREEMPT_EW_PHASE); }
        else { if (timeUp(T_MIN_GREEN)) enterPhase(EW_Y_PHASE); }
        break;
      case NS_Y_PHASE:
      case EW_Y_PHASE:
        if (timeUp(T_YELLOW)) enterPhase(ALL_RED_PHASE);
        break;
      case ALL_RED_PHASE:
        if (timeUp(T_ALL_RED)) {
          if (pendingPreempt == PRE_NS) enterPhase(PREEMPT_NS_PHASE);
          else enterPhase(PREEMPT_EW_PHASE);
        }
        break;
      default: break; // let ped/preempt sequences complete
    }
  }

  // FSM
  switch(phase){
    case NS_G_PHASE: {
      if (timeUp(greenBudget)) {
        if (occNS() && (greenBudget + T_EXT_STEP) <= T_MAX_GREEN) {
          greenBudget += T_EXT_STEP;
        } else if (timeUp(T_MAX_GREEN) || !occNS()) {
          enterPhase(NS_Y_PHASE);
          if (greenBudget >= T_MAX_GREEN) { maxHitsNS++; } else maxHitsNS = 0;
        }
      }
      if (maxHitsNS >= 2 && timeUp(T_MIN_GREEN)) {
        enterPhase(NS_Y_PHASE); maxHitsNS = 0;
      }
      break;
    }
    case NS_Y_PHASE:
      if (timeUp(T_YELLOW)) enterPhase(ALL_RED_PHASE);
      break;

    case EW_G_PHASE: {
      if (timeUp(greenBudget)) {
        if (occEW() && (greenBudget + T_EXT_STEP) <= T_MAX_GREEN) {
          greenBudget += T_EXT_STEP;
        } else if (timeUp(T_MAX_GREEN) || !occEW()) {
          enterPhase(EW_Y_PHASE);
          if (greenBudget >= T_MAX_GREEN) { maxHitsEW++; } else maxHitsEW = 0;
        }
      }
      if (maxHitsEW >= 2 && timeUp(T_MIN_GREEN)) {
        enterPhase(EW_Y_PHASE); maxHitsEW = 0;
      }
      break;
    }
    case EW_Y_PHASE:
      if (timeUp(T_YELLOW)) enterPhase(ALL_RED_PHASE);
      break;

    case ALL_RED_PHASE: {
      if (!timeUp(T_ALL_RED)) break;
      if (servedNSLast) {
        if (pedReqEW) { pedReqEW = false; servePedEWWalkFDW(); }
        else { enterPhase(EW_G_PHASE); servedNSLast = false; }
      } else {
        if (pedReqNS) { pedReqNS = false; servePedNSWalkFDW(); }
        else { enterPhase(NS_G_PHASE); servedNSLast = true; }
      }
      break;
    }

    case PED_NS_WALK:
      if (timeUp(T_PED_WALK)) enterPhase(PED_NS_FDW);
      break;
    case PED_NS_FDW:
      if (((millis()/T_PED_FLASH) % 2) == 0) allRed();
      if (timeUp(T_PED_FDW)) { enterPhase(EW_G_PHASE); servedNSLast = false; }
      break;

    case PED_EW_WALK:
      if (timeUp(T_PED_WALK)) enterPhase(PED_EW_FDW);
      break;
    case PED_EW_FDW:
      if (((millis()/T_PED_FLASH) % 2) == 0) allRed();
      if (timeUp(T_PED_FDW)) { enterPhase(NS_G_PHASE); servedNSLast = true; }
      break;

    case PREEMPT_NS_PHASE:
      if (timeUp(T_PREEMPT_GREEN)) { pendingPreempt = PRE_NONE; enterPhase(PREEMPT_HOLD_PHASE); }
      break;
    case PREEMPT_EW_PHASE:
      if (timeUp(T_PREEMPT_GREEN)) { pendingPreempt = PRE_NONE; enterPhase(PREEMPT_HOLD_PHASE); }
      break;

    case PREEMPT_HOLD_PHASE:
      if (timeUp(T_PREEMPT_HOLD)) {
        if (lastPreempt == APP_NS) { enterPhase(EW_G_PHASE); servedNSLast = false; }
        else if (lastPreempt == APP_EW) { enterPhase(NS_G_PHASE); servedNSLast = true; }
        else { if (servedNSLast) { enterPhase(EW_G_PHASE); servedNSLast = false; }
               else { enterPhase(NS_G_PHASE); servedNSLast = true; } }
        lastPreempt = APP_NONE;
      }
      break;
  }

  // Telemetry (every ~0.5 s)
  static unsigned long tPrint = 0;
  if (millis() - tPrint > 500) {
    tPrint = millis();
    Serial.print("occNS="); Serial.print(occNS());
    Serial.print(" occEW="); Serial.print(occEW());
    Serial.print(" pedNS="); Serial.print(pedReqNS);
    Serial.print(" pedEW="); Serial.print(pedReqEW);
    Serial.print(" pre=");   Serial.print((pendingPreempt==PRE_NS)?'N':(pendingPreempt==PRE_EW)?'E':'-');
    Serial.print(" phase="); Serial.println(phase);
  }

  delay(10);
}