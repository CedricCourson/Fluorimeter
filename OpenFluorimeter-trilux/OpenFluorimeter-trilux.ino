/*
 *  ---------- FLUORIMETER ------------------
 *  www.openoceanography.org
 *  Fluorimetre open source pour le projet OPEN LAGOON.
 *  
 *  Cet instrument mesure :
 *  Chlorophyle 
 *  Turbidité
 *  Température eau de mer
 *  Salinité eau de mer (coductivité)
 *  Pression
 *  + température interne du boitier
 *  
 * Etat: 1.6 = tout fonctionne, ok
 * fichier enregitré dans un fomat "data1002.csv" a chaque allumage
 * 
 * Reste a faire : 
 * - eventuellement ajouter la lecture des infos battery dans une fonction
 * - eventuellement voir pour le retours a la ligne de la trilux induit un saut de ligne entre chaque ligne de mesure du CSV
 * - eventuellement ajouter calcul de salinité....
 */



// ---------------------   PARAMETRES MODIFIABLE DU PROGRAM    -----------------------------------
// Version et numero de serie
char numserie[] = "OSOFL21001";      // Numero de serie de la sonde
char versoft[] = "1.6";             // version du code

#define TIME_TO_SLEEP  900           // Durée d'endormissement entre 2 cycles complets de mesures (in seconds) par défault 900 = 15min
// int nbrMes = 1;                     // nombre de mesure de salinité et température par cycle par déafuatl 3
int nbr_mes_cycle = 30;              // nombre de mesure dans un cycle
//int delay_mes = 1;                  // delay entre chaque mesure d'un cycle (en s)
// --------------------     FIN DES PARAMETRES MODIFIABLES     -----------------------------------




// Library
#include <GxEPD.h>                      // Epaper Screen
//#include <GxGDEW042T2/GxGDEW042T2.h>    // Epaper Screen 4.2" b/w
#include <GxGDEH029A1/GxGDEH029A1.h>    // Epaper Screen 2.9" b/w
#include <Fonts/FreeMonoBold9pt7b.h>    // font for epaper sreen
#include <Fonts/FreeMonoBold12pt7b.h>   // font for epaper sreen
#include <Fonts/FreeMonoBold18pt7b.h>   // font for epaper sreen
#include <Fonts/FreeMonoBold24pt7b.h>   // font for epaper sreen
#include <GxIO/GxIO_SPI/GxIO_SPI.h>     // epaper sceeen
#include <GxIO/GxIO.h>                  // epaper screen
#include <Wire.h>                       //enable I2C.
#include <DS3231.h>                     // Pour horloge RTC
#include "SPI.h"                        // pour connection ecran bus SPI
#include <SD.h>                         // pour carte SD
#include "MS5837.h"                     // pour capteur de pression blue robotics 
#include <HardwareSerial.h>             // pour capteur serie fluo
#include <SparkFunBQ27441.h>            // Batterie fuel gauge lipo babysister
#include <OneWire.h>                    // pour capteur ds18b20
#include <DallasTemperature.h>          // pour capteur ds18b20

//SPI pin definitions pour ecran epaper
GxIO_Class io(SPI, /*CS=5*/ 0, /*DC=*/ 13, /*RST=*/ 10); // arbitrary selection of 17, 16  //SS remplacé par 0
GxEPD_Class display(io, /*RST=*/ 10, /*BUSY=*/ 4); // arbitrary selection of (16), 4

// définition pour les fonts ecran
const GFXfont* f1 = &FreeMonoBold9pt7b;
const GFXfont* f2 = &FreeMonoBold12pt7b;
const GFXfont* f3 = &FreeMonoBold18pt7b;
const GFXfont* f4 = &FreeMonoBold24pt7b;

//déclaration pour horloge
DS3231 Clock;
bool Century=false;
bool h12;
bool PM;
//byte ADay, AHour, AMinute, ASecond, ABits;
//bool ADy, A12h, Apm;
//byte year, month, date, DoW, hour, minute, second;
String second,minute,hour,date,month,year; 
String datenum, timenum;                   // pour format de date en 1 seule écriture
String datetime;

// Set BATTERY_CAPACITY to the design capacity of your battery.
const unsigned int BATTERY_CAPACITY = 3400; // e.g. 3400mAh battery
unsigned int soc ;
unsigned int volts; 

// variable fichier pour SD
String datachain = "";                   // chaine de donnée texte de mesure   

//déclaration pour la gestion de fichier sur la carte SD
String filename, filename_temp, str_index, filetrans;
int ind;                  // index vérification de fichier

//déclaration pour capteur de pression
MS5837 sensor;
float  wat_pressure, wat_temp, wat_depth, alt;

// déclaration pour capteur de fluo
#define RXD1 16 //25
#define TXD1 17 //26
HardwareSerial Serial1(1); // si 25/26 alors Serial2(2), et modifier tout les Serial1 par Serial2.
String fluochain; 
String datafluo;
int c;
const char marqueurDeFin = '\r';
int pinFluo =27;
int delay_chauffe = 10; // delay de mise en route de la trilux en s
int StartStock = 0;

// definition pour la fonction deepsleep de l'ESP32
#define uS_TO_S_FACTOR 1000000      // Conversion factor for micro seconds to seconds
RTC_DATA_ATTR int bootCount = 0;    // utile pour enregistrer un compteur dans la memoire rtc de l'ULP pour un compteur permettant un suivi entre chaque veille
RTC_DATA_ATTR int filenumber = 0;      // nom de fichier pour le garder entre l'initialisation (1er boot) et les mesures (tout les autres boots)

// definition pour les carte Atlas
#define ecAddress 100 
//#define rtdAdress 102
//byte rtdCode=0;                     // Used to hold the I2C response code. 
//byte rtdInChar=0;                   // Used as a 1 byte buffer to store in bound bytes from the RTD Circuit.   
//char rtdData[20];                   // We make a 20 byte character array to hold incoming data from the RTD circuit. 
//int rtdDelay = 600;                 // Used to change the delay needed depending on the command sent to the EZO Class RTD Circuit. 600 by default. It is the correct amount of time for the circuit to complete its instruction.
byte ecCode = 0;                    // Used to hold the I2C response code.
byte ecInChar = 0;                  // Used as a 1 byte buffer to store in bound bytes from the EC Circuit.
char ecData[48];                    // We make a 48 byte character array to hold incoming data from the EC circuit.
int ecDelay = 600;                  // Used to change the delay needed depending on the command sent to the EZO Class EC Circuit. 600 par defaut. It is the correct amount of time for the circuit to complete its instruction.
char *ec;                           // Char pointer used in string parsing.
char *tds;                          // Char pointer used in string parsing.
char *sal;                          // Char pointer used in string parsing.
char *sg;                           // Char pointer used in string parsing.
int ecpin =12;                      // pin for turn on/off the EC card

// déclaration initiale des variables de la fonction cal_sal (Aminot A., Kérouel R. (2004), Hydrologie des écosystèmes marins. Paramètres et analyses. Cf pages 74-78)
int Rp = 1;             
float aa[] = {0.0080, -0.1692, 25.3851, 14.0941, -7.0261, 2.7081};
float bb[] = {0.0005, -0.0056, -0.0066, -0.0375, 0.0636, -0.0144};
float cc[] = {0.6766097, 0.0200564, 0.0001104259, -6.9698E-07, 1.0031E-09};
float k = 0.0162;
float rt, R_t, S, modA, modB;

//déclaration pour capteur de température interne ds18b20
#define ONE_WIRE_BUS 2
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
float tempint;





void setup() {

  // -------------------------------------------------------------------------------------------------------------------
  // ------------------        HERE IS NEEDED THE PARAMETER FOR THE ENTIER PROGRAMM   ----------------------------------
  // -------------------------------------------------------------------------------------------------------------------
  
 
  Serial.begin(115200);                        // communication PC
  Serial1.begin(9600, SERIAL_8N1, RXD1, TXD1); // communication serie avec la sonde de fluo
  Wire.begin();                                // for I2C communication
  setupBQ27441();                              // for Lipo Babysister sparkfun lipo fuelgauge
  display.init();                              // enable display Epaper
  delay(500); //Take some time to open up the Serial Monitor and enable all things  

  pinMode(pinFluo, OUTPUT);          // define pin fluo
  pinMode(ecpin, OUTPUT);            // define pin EC
  digitalWrite(pinFluo, LOW);        // turn off the pin fluo          
  digitalWrite(ecpin, LOW);          // turn off the ec 



  // Test carte SD
  Serial.print("Initializing SD card...");   
    if (!SD.begin(5)) {                      // // see if the card is present and can be initialized, ajouter ici chipSelect ou 5 pour la pin 5 par default
    Serial.println("Card failed, or not present");
    errormessage();
    // don't do anything more:
    return;
  }
  Serial.println("card initialized."); 


  if(bootCount == 0) //Run this only the first time
  {
    // -----------------------------------------------------------------------------------------------------------
    // --------------        HERE IS ONLY THE INTRODUCION           ----------------------------------------------
    // -----------------------------------------------------------------------------------------------------------
    
    affichageintro();
    delay (2000);
    
    //création nom de fichier :
          
        // création du nom de fichier et écriture sur la carte SD en automatisant la vérification de fichiers existant.
        filename =""; filename +="/data"; 
        str_index=String(1000);                                    
        filename_temp = filename + str_index + ".csv";              //1er fichier du jour
        int ind=0;
  
          while(SD.exists(filename_temp)){                          // vérifie l'existantce de fichier dans un boucle en indexant le numéro de fichier
          ind++;      
          str_index=String(1000+ind); 
          filename_temp = filename + str_index + ".csv";
          }
        filename = filename_temp;                                   // création du nom de fichier final  

        filenumber = str_index.toInt();                             //convertion du nom de fichier en int pour transfert en memoire RTC
        
    // fin de création de fichier
  
  
    // Write the 1st ligne of the CSV file
        datachain += "Date & heure "; datachain += " ; ";
        datachain += "Bat %"; datachain += " ; "; datachain += "Bat mV"; datachain += " ; ";
        datachain += "Intern Temp (C)"; datachain += " ; ";
        datachain += "Profondeur(m)"; datachain += " ; ";   datachain += "Temperature mer (C)"; datachain += " ; ";
        datachain += "Conductivity(ms/cm)"; datachain += ";"; 
        datachain += "Fluochain"; datachain += ";";
  
    // print the datachain on the serial port
        Serial.println("Format de la chaine enregistrée : ");
        Serial.println(datachain);
  
    // save the datachain on the SD card
        File dataFile = SD.open(filename, FILE_APPEND);
        if (dataFile) {                                        // if the file is available, write to it:
          dataFile.println(datachain);
          dataFile.close();
          Serial.println("Fichier créer avec succés");
          Serial.print("Filename : "); Serial.println(filename);
        }
        else {                                                 // if the file isn't open, pop up an error:
          Serial.println("error opening file"); 
          errormessage();
          delay(3000);   
        }

      delay(1000);

      message1();
      
      bootCount = bootCount+1;   // change the bootcount number to skip the initial step after wake up

  
  }else
  {
      // ----------------------------------------------------------------------------
      // ----------------------- HERE IS THE MAIN LOOP ------------------------------
      // ----------------------------------------------------------------------------

      // Turn in Sensors
            digitalWrite(pinFluo, HIGH);               // turn on the trilux sensor 
            delay(delay_chauffe*1000);                 // give time to the trilux sensor to start (min 10 seconds)
           
            digitalWrite(ecpin, HIGH);                 // Turn on the EZO-EC card
            delay(1000);     

      // Initialize pressure Sensor
            while (!sensor.init()) {                                            // Returns true if initialization was successful
              Serial.println("Init failed!");                                     // We can't continue with the rest of the program unless we can initialize the sensor
              Serial.println("Are SDA/SCL connected correctly?");
              Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
              Serial.println("\n\n\n");
              delay(5000);
            }
            sensor.setModel(MS5837::MS5837_30BA);
            sensor.setFluidDensity(1029); // kg/m^3 (1029 for seawater, 997 for freshwater)

      // on récupére le nom de fichier
            Serial.print("File number : "); Serial.println(filenumber);
            filename = ""; filename += "/data"; filename += String(filenumber); filename += ".csv";
            Serial.print("File name : ");Serial.println(filename);
            Serial.println(" ");

      // Do Measures and save it on SD card !!!!!!!!!!!!!
            for(int i=0; i<nbr_mes_cycle;i++){
                Serial.print("---CYCLE--- : ");Serial.println(i+1);

                // Read battery stats from the BQ27441-G1A
                unsigned int soc = lipo.soc();  // Read state-of-charge (%)
                unsigned int volts = lipo.voltage(); // Read battery voltage (mV)
                Serial.println("--- Battery sensor");
                Serial.print(" Charge % :"); Serial.println(soc);
                Serial.print(" Tension mV : "); Serial.println(volts);
      
                // lecture des différents capteurs
                lecture_rtc();            // lecture de l'horloge rtc
                mes_temp_int();           // temperature interne du boitier
                mesure_pressure();        // measure pressure/temp
                mesureEC();               // mesure de la conductivité EC
                mes_fluo();               // on lit la chaine et on l'enregistre dans la variable datafluo
             
                // Création de la chaine à enregistrer sur la carte SD
                String datachain ="";
                datachain += datetime; datachain += " ; ";
                datachain += soc; datachain += " ; "; datachain += volts ; datachain += " ; ";
                datachain += tempint; datachain += ";";
                datachain += wat_depth; datachain += " ; "; datachain += wat_temp; datachain += " ; "; 
                datachain += ecData; datachain += ";";
                datachain += datafluo; //datachain += ";";
                  
                // enregistrement sur la carte SD
                  File dataFile = SD.open(filename, FILE_APPEND);         
                  if (dataFile) {                                        // if the file is available, write to it:
                    dataFile.println(datachain);
                    dataFile.close();
                    Serial.println("ok : Datachain saved on SD card !");
                  }
                  else {                                                 // if the file isn't open, pop up an error:
                    Serial.println("error opening file");  
                    errormessage();
                    delay(3000);  
                  }
                  
                // Affichage port serie des datas
                  Serial.println("Datachain : "); Serial.println(datachain); //affichage de la chaine complete sur le port serie
                  Serial.println(" ");
                  Serial.println("------------------------------------");
                  Serial.println(" ");             
            }
      
      // print data on screen
            display.setRotation(3);
            display.fillScreen(GxEPD_WHITE);
            
            display.setTextColor(GxEPD_BLACK);
            display.setFont(f1);
            display.setCursor(5, 10); display.print("Depth :");display.print(wat_depth); display.print(" m");
            display.setCursor(5,30); display.print("Sea T :");display.print(wat_temp); display.print(" deg C");   
            display.setCursor(5,50); display.print("Sea EC:");display.print(ecData); display.print(" mS/cm");
            display.setCursor(5,70); display.print("Fluo  :");display.print(datafluo);
          
            //cadre
            //display.fillRect(147, 0, 2, 90, GxEPD_BLACK);
            display.fillRect(0, 94, 296, 2, GxEPD_BLACK); 
            
            unsigned int soc = lipo.soc();  // Read state-of-charge (%)
            //Last update
            display.setFont(f1);
            display.setCursor(0,110); display.print(datetime);
            //display.setCursor(0,110); display.print(date);display.print("/");display.print(month);display.print("/");display.print(year);  //date
            //display.setCursor(123,110);display.print(hour);display.print("h");display.print(minute);                                       //heure
            display.setCursor(210,110);display.print("Bat:");display.print(soc);display.print("%");
            display.setCursor(0,124); display.print("Temp interne : "); display.print(tempint);display.print(" deg C");
            //      display.setCursor(0, 125); display.print("Lat:");display.print(gps.location.lat(), 5);                                                     //lattitude
            //      display.setCursor(148,125); display.print("Lng:");display.print(gps.location.lng(), 4);                                                   //longitude
            display.update();

     // Turn off sensors
           digitalWrite(pinFluo, LOW);
           digitalWrite(ecpin, LOW);
           delay (500);
  }

  // endormissement esp32
  Serial.println("Going to sleep");
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  esp_deep_sleep_start();
 
}


void loop() {
 //never use because of the sleeping of the princess esp32
}



/*
 * -----------------------------------------------------------------------------------------------------------------------------------------------------
 * -------------------- FUNCTIONS CALLED IN THE PROGAM -------------------------------------------------------------------------------------------------
 * -----------------------------------------------------------------------------------------------------------------------------------------------------
 */




void affichageintro(){                                  // texte intro à l'allumage, juste pour faire jolie et afficher éventuellement les versions du programme
  display.setRotation(3);
  display.fillScreen(GxEPD_WHITE);
  
  // subtitle 
  display.setTextColor(GxEPD_BLACK);
  display.setFont(f1);
  display.setCursor(5, 10  );
  display.println("OpenSourceOceanograpy");
  
  //titre
  display.setFont(f3);
  display.setCursor(25, 55  );
  display.println("Fluorimeter");

  //version
  display.setFont(f1);
  display.setCursor(30, 90);
  display.print("Num Serie :");display.println(numserie);
  display.setCursor(30, 110);
  display.print("Ver Soft  :");display.print(versoft);
 
  display.update();
}


void message1(){                                  // texte intro à l'allumage, juste pour faire jolie et afficher éventuellement les versions du programme
  display.setRotation(3);
  display.fillScreen(GxEPD_WHITE);
  
  // subtitle 
  display.setTextColor(GxEPD_BLACK);
  display.setFont(f1);
  display.setCursor(0, 20); display.println("Startup file & SD : ok");
  display.setCursor(0, 40); display.print("File name :");display.print(filename);

  display.setCursor(0,90); display.print("Acquisition every ");display.print(TIME_TO_SLEEP/60);display.print(" min");
  display.setCursor(0,110); display.print("Cycle by acquisition :");display.print(nbr_mes_cycle);

  display.update();
}




void errormessage(){                                       // message d'erreur en cas d'impossibilité de lire la carte SD

  Serial.println("Error : error with sd card");
  display.setRotation(3);
  display.fillScreen(GxEPD_WHITE);
  
  //titre
  display.setTextColor(GxEPD_BLACK);
  display.setFont(f3);
  display.setCursor(25, 55  );
  display.println("  ERROR ");

  //version
  display.setFont(f1);
  display.setCursor(30, 90);
  display.print("No SD Card ");
 
  display.update();
}


void mesure_pressure(){                               // fonction mesure de la pression et température sur le capteur blue robotics
  // Update pressure and temperature readings
  sensor.read();

  wat_pressure = sensor.pressure();
  wat_temp=sensor.temperature();
  wat_depth=sensor.depth();
  alt=sensor.altitude();

  Serial.println("--- Pressure Sensor :");
  Serial.print("Pressure: "); 
  Serial.print(wat_pressure); 
  Serial.println(" mbar");
  
  Serial.print("Temperature: "); 
  Serial.print(wat_temp); 
  Serial.println(" deg C");
  
  Serial.print("Depth: "); 
  Serial.print(wat_depth); 
  Serial.println(" m");
  
  Serial.print("Altitude: "); 
  Serial.print(alt); 
  Serial.println(" m above mean sea level");
  //Serial.println(" ");
}



void mes_fluo(){
  while(Serial1.available()){
    c = Serial1.read();
    if(StartStock==1){
        switch (c) {
          case marqueurDeFin:
          //Serial.print("fluochain : "); Serial.println(fluochain);
          datafluo = fluochain;
          fluochain="";
          StartStock=0;
          default:
          if(c!=marqueurDeFin){fluochain += char(c);}
        }
    }
    if(c==marqueurDeFin){
      StartStock = 1;
    }
  }

  Serial.println("--- Fluo Sensor :");
  Serial.print("datafluo : "); Serial.println(datafluo); // juste pour voir la valeur de datafluo
}


void lecture_rtc(){                           // fonction lecture de l'horloge RTC et ecriture dans les variables utilent
      
      int sec=Clock.getSecond();
      if(sec<10){second = String(0) + String(sec);}
      else{second = sec;}
      int minu=Clock.getMinute();
      if(minu<10){minute = String(0) + String(minu);}
      else{minute = minu;}
      int heure=Clock.getHour(h12, PM);
      if(heure<10){hour = String(0) + String(heure);}
      else{hour = heure;}
      int jour=Clock.getDate();
      if(jour<10){date = String(0) + String(jour);}
      else{date = jour;}
      int mois=Clock.getMonth(Century);
      if(mois<10){month = String(0) + String(mois);}
      else{month = mois;}
      year=Clock.getYear();
      datenum =""; datenum +=year; datenum +=month; datenum +=date;
      timenum =""; timenum +=hour; timenum +=minute; timenum +=second;
      datetime="";datetime+=date;datetime+="/";datetime+=month;datetime+="/20";datetime+=year;datetime+=" ";datetime+=hour;datetime+=":";datetime+=minute;datetime+=":";datetime+=second;

      Serial.println("--- RTC values :");
      Serial.print("Date :"); Serial.println(datenum);
      Serial.print("Time :"); Serial.println(timenum);
      Serial.print("DateTime"); Serial.println(datetime);
      //Serial.print(" ");
}    









void mesureEC(){
    Serial.println("--- EC Sensor :");
    Wire.beginTransmission(ecAddress);   // Call the circuit by its ID number.
    Wire.write('r');                     // r for reading sensor
    Wire.endTransmission();              // End the I2C data transmission. 
    delay(ecDelay);                      // Reading time needing, 600 by default
    Wire.requestFrom(ecAddress, 48, 1);  // Call the circuit and request 48 bytes (this is more than we need)
    ecCode = Wire.read();                // The first byte is the response code, we read this separately.
    byte i = 0;                          // Counter used for EC_data array. 
    while (Wire.available()) {           // Are there bytes to receive.
      ecInChar = Wire.read();            // Receive a byte.
      ecData[i] = ecInChar;              // Load this byte into our array.
      i += 1;                            // Incur the counter for the array element.
      if (ecInChar == 0) {               // If we see that we have been sent a null command.
        i = 0;                           // Reset the counter i to 0.
        Wire.endTransmission();          // End the I2C data transmission.
        break;                           // Exit the while loop.
      }
    }
    switch (ecCode) {                    // Switch case based on what the response code is.
      case 1:                            // Decimal 1.
        Serial.println("EC Success");    // Means the command was successful.
        break;                           // Exits the switch case.
      case 2:                            // Decimal 2.
        Serial.println("EC Failed");     // Means the command has failed.
        break;                           // Exits the switch case.
      case 254:                          // Decimal 254.
        Serial.println("EC Pending");    // Means the command has not yet been finished calculating.
        break;                           // Exits the switch case.
      case 255:                          // Decimal 255.
        Serial.println("EC No Data");    // Means there is no further data to send.
        break;                           // Exits the switch case.
    }
    ec = strtok(ecData, ","); 
    tds = strtok(NULL, ","); 
    sal = strtok(NULL, ","); 
    sg = strtok(NULL, ",");    // Let's pars the string at each comma.

    Serial.print("EC value : "); Serial.println(ecData); 
//  Serial.print("EC:");                //we now print each value we parsed separately.
//  Serial.println(ec);                 //this is the EC value.
//  Serial.print("TDS:");               //we now print each value we parsed separately.
//  Serial.println(tds);                //this is the TDS value.
//  Serial.print("SAL:");               //we now print each value we parsed separately.
//  Serial.println(sal);                //this is the salinity value.
//  Serial.print("SG:");                //we now print each value we parsed separately.
//  Serial.println(sg);                 //this is the specific gravity.
}



//void cal_sal(float t, float C){         // fonction sinplifié à intégrer dans un code
//  // calcul intermédiaire 
//  int i;
//  rt=0;                                 // réinitilisation entre chaque boucle
//  for (i=0; i<5; i++){                  // calcul de rt
//    rt += cc[i]*pow(t,float(i));
//  }
//
//  R_t= C /(42.914*rt);                  // calcul de R_t
//
//  modA =0; modB=0;                      // réinitilisation entre chaque boucle
//  for (i=0; i<6;i++){                   // cal modA et modB
//    modA += aa[i]*pow(R_t,float(i/2));
//    modB += bb[i]*pow(R_t,float(i/2));
//  }
//
//  //cal salinité
//  S = modA+((t-15)/(1+k*(t-15)))*modB;
//  Serial.print("Salinité = "); Serial.println(S,4);
//}



void mes_temp_int(){    // température interne du boitier pour info
  sensors.begin();
  delay(500);
  sensors.requestTemperatures(); // Send the command to get temperatures 
  tempint = sensors.getTempCByIndex(0);
  Serial.println("--- Interne Temp sensor :");
  Serial.print("Temperature : ");
  Serial.println(tempint, 3);   // We use the function ByIndex, and as an example get the temperature from the first sensor only.
    
}




void setupBQ27441(void)
{
  // Use lipo.begin() to initialize the BQ27441-G1A and confirm that it's
  // connected and communicating.
  if (!lipo.begin()) // begin() will return true if communication is successful
  {
  // If communication fails, print an error message and loop forever.
    Serial.println("Error: Unable to communicate with BQ27441.");
    Serial.println("  Check wiring and try again.");
    Serial.println("  (Battery must be plugged into Battery Babysitter!)");
    while (1) ;
  }
  Serial.println("Connected to BQ27441!");
  
  // Uset lipo.setCapacity(BATTERY_CAPACITY) to set the design capacity
  // of your battery.
  lipo.setCapacity(BATTERY_CAPACITY);
}
