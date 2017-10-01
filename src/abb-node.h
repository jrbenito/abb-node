void txRadio(Message * mess);
void readUptime(Message *mess);
void readTXInt(Message *mess);
void writeTXInt(const Message *mess);
void readRSSI(Message *mess);
void readVoltage(Message *mess);
void readACK(Message *mess);
void writeACK(const Message *mess);
void readToggle(Message *mess);
void writeToggle(const Message *mess);
void sleep();
void watchdogSetup(void);
void buttonHandler();

enum DSP_Registers {
    gridVoltage   = 1,
    gridPower     = 3,
    gridFrequency = 4,
    powerIn       = 8,
    inverterTemp  = 21,
    boosterTemp, 
    inputVoltage,
    inputCurrent   = 25,
    gridVoltageAvg = 32,
    powerPeak      = 34,
    powerPeakDay,
    heatsinkTemp   = 49
};

typedef struct {
    float gridVoltage;
    float gridPower;
    float gridFrequency;
    float powerIn;
    float inverterTemp;
    float boosterTemp;
    float inputVoltage;
    float inputCurrent;
    float gridVoltageAvg;
    float powerPeak;
    float powerPeakDay;
    float heatsinkTemp;
} InverterRegisters;

// devices in the node
void readLED(Message *mess);
void writeLED(const Message *mess);
void readRelay(Message *mess);
void writeRelay(const Message *mess);
void readGV(Message *mess);
void readGP(Message *mess);
void readGF(Message *mess);
void readPwrIn(Message *mess);
void readInvTemp(Message *mess);
void readInputV(Message *mess);
void readInputA(Message *mess);
void readPPKDay(Message *mess);
