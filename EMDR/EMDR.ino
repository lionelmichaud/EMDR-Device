/*
 Test du MM5451 Avec Arduino
 http://www.Actuino.fr
 Version Simple avec digitalWrite
 Sans contrôle de luminosité
 */

#define DEBUG  // commenter en mode production
//#define POT_BRIGHT // présence d'un potar de réglage de la luminosité
#define POT_SPEED  // présence d'un potar de réglage de la vitesse
#define DELAY_LED 10 // (min = 1us) x (0 ... 100) pas sur le nb de microseconde d'attente entre deux commande successive aux LEDs

#define PIN_CLK  14 // La broche CLK  du 5451
#define PIN_DATA 27 // la broche DATA du 5451

#define PIN_OUT_BRIGHT 12 // la broche de controle Brightness
#define PIN_IN_BRIGHT  13 // la broche de réglage Brightness
#define PIN_IN_SPEED   15 // la broche de réglage vitesse de défilement
#define PIN_LED        22 // la broche LED bleue

// setting PWM properties
const int freq          = 1000;
const int resolution    = 8;
const int brightChannel = 0;

// niveau de luminosité
const int defBrightness = 100; // % valeur par défaut
unsigned int brightness = 100; // % valeur courante

// niveau de vitesse
const int defSpeed = 100; // % valeur par défaut
unsigned int speed = 100; // % valeur courante

// commande des LEDs
uint32_t leds;
int nbLed         = 1; // nbr de LED allumées de 1 à 3 (si + alors 3)
boolean toTheLeft = true;

// variables
int value = 0;

//===========================================================
void setup() {
    // initialisation des variables
    brightness = defBrightness;
    speed = defSpeed;
    switch (nbLed) {
        case 1:
            leds = 1; // 10000000
            break;
        case 2:
            leds = 3; // 11000000
            break;
        default:
            leds = 7; // 11100000
            break;
    }
    
    // Initialisation des broches
    pinMode(PIN_CLK,    OUTPUT);
    pinMode(PIN_DATA,   OUTPUT);
    pinMode(PIN_LED,    OUTPUT);
    pinMode(PIN_OUT_BRIGHT, OUTPUT);
    pinMode(PIN_IN_BRIGHT,  INPUT);
    pinMode(PIN_IN_SPEED,   INPUT);
    
    // broches à LOW
    digitalWrite(PIN_CLK,    LOW);
    digitalWrite(PIN_DATA,   LOW);
    digitalWrite(PIN_LED,    LOW);
    // broches à HIGH
    digitalWrite(PIN_OUT_BRIGHT, HIGH);
    
#ifdef POT_BRIGHT
    // PWM pourle controle de brightness
    ledcSetup(brightChannel, freq, resolution);
    
    // attach the channel to the GPIO to be controlled
    ledcAttachPin(PIN_OUT_BRIGHT, brightChannel);
    ledcWrite(brightChannel, int((float(brightness)/100.0)*255));
#endif

    // initialisation de la liaison série
    Serial.begin(115200);
    delay(1000);
    
    // allumer la LED bluee pour signifier la fin de l'initialisation
    digitalWrite(PIN_LED, HIGH);
    
}

// envoie un seul bit au 5451
//---------------------------
void sendBit(boolean b) {
    // CLK est forcément à LOW.
    digitalWrite(PIN_DATA, b);   // on ajuste le bit DATA
#ifdef DEBUG
    Serial.print(b);
#endif
    delayMicroseconds(20); // _delay_us(1); // attendre 1µs
    digitalWrite(PIN_CLK, HIGH);   // Monter CLK
    delayMicroseconds(100); // _delay_us(2); // attendre 2µs
    digitalWrite(PIN_CLK, LOW);   // Redescendre CLK
}

// envoie un octet au 5451
//---------------------------
void sendByte(uint8_t a) {
#ifdef DEBUG
    Serial.print("byte: " + String(a, BIN) + " => ");
#endif
    for (byte i=0;i<8;i++) {
        sendBit(boolean((a>>i) & 0x1)); // on envoie le LSB
    }
#ifdef DEBUG
    Serial.println();
#endif
    /*
     shiftOut(PIN_DATA,PIN_CLK,LSBFIRST,a);
     //digitalWrite(PIN_CLK, LOW);   // Redescendre CLK
     */
}

// envoie 4 octets, soit 4x8=32 bits.
// les 3 derniers seront envoyés à 0
// on envoie le LSB en premier.
// donc le LSB de d jusque MSB de d, puis c.
// on termine par a puis les 3 zéros
//---------------------------------------------
void send4Bytes(uint8_t a,uint8_t b,uint8_t c,uint8_t d) {
    sendBit(1); // On commence par envoyer un bit de start, "1"
    
    sendByte(d);
    sendByte(c);
    sendByte(b);
    sendByte(a);
    
    sendBit(0); // Les 3 derniers bits à 0
    sendBit(0);
    sendBit(0);
}

// envoie 32 bits au 5451, soit un unsigned 32 integer
//----------------------------------------------------
void send32Bits(uint32_t a) {
    for (uint32_t i=0;i<32;i++) {
        sendBit(boolean((a>>i) & 0x1)); // on envoie le LSB
    }
    /*
     shiftOut(PIN_DATA,PIN_CLK,LSBFIRST,a);
     //digitalWrite(PIN_CLK, LOW);   // Redescendre CLK
     */
}

// envoie 32 bits, soit unsigned 32 int.
// les 3 derniers seront envoyés à 0
// on envoie le LSB en premier.
//---------------------------------------------
void sendLedCommand(uint32_t command) {
    sendBit(1); // On commence par envoyer un bit de start, "1"
    
    send32Bits(command);
    
    sendBit(0); // Les 3 derniers bits à 0
    sendBit(0);
    sendBit(0);
}

//===========================================================
void loop() {
    // attendre XX µs
    delayMicroseconds((100 - speed) * DELAY_LED);
    
    // lecture du potar de réglage et commande de brightness
#ifdef POT_BRIGHT
    value = analogRead(PIN_IN_BRIGHT);
    brightness = map(value, 0, 4095, 0, 100);
    ledcWrite(brightChannel, int((float(brightness)/100.0)*255));
#ifdef DEBUG
    Serial.println("brightness = " + String(brightness));
    Serial.println("brightness level = " + String(int((float(brightness)/100.0)*255)));
#endif
#else
    // TODO: ajouter le code ici
    // la valeur de brightness reste constante et égale à sa valeur initiale defBrightness = 100%
#endif
    
    // lecture du potar de réglage de vitesse
#ifdef POT_SPEED
    value = analogRead(PIN_IN_SPEED);
    speed = map(value, 0, 4095, 0, 100);
#ifdef DEBUG
    Serial.println("speed = " + String(speed));
#endif
#else
    // TODO: ajouter le code ici
    // la valeur de speed reste constante et égale à sa valeur initiale defSpeed = 100%
#endif
    
    // commande des LEDs
#ifdef DEBUG
    while (1) {
        Serial.println("Tout éteint");
        send4Bytes(0,0,0,0); // Tout eteint
        delay(10000);
        Serial.println("Une LED sur deux");
        send4Bytes(B10101010,B10101010,B10101010,B10101010); // une LED sur deux
        delay(10000);
        Serial.println("Tout allumé");
        send4Bytes(255,255,255,255); // Tout allumé
        delay(10000);
    }
#else
    // commander les LED
    sendLedCommand(leds);
    
    // décaler la commande des LED d'un cran vers la gauche ou la droite
    (toTheLeft ? leds <<= leds : leds >>= leds)
    
    // inverser le sens de défilement quand on arrive en butée
    if toTheLeft && (boolean((leds>>31) & 0x1)) // on arrive en butée à gauche, repartir vers la droite
        toTheLeft = false;
    else if !toTheLeft && (boolean(leds & 0x1)) // on arrive en butée à droite, repartir vers la gauche
        toTheLeft = true;
#endif
}
