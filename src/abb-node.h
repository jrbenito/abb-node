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

// devices in the node
void readRelay(Message *mess);
void writeRelay(const Message *mess);

