/*
 * Fonctionne trés bien avec capteur de pression et température
 * Reste à intégrer la sonde fluo et eventuellement salinité ou O2....
 * 
 */


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

//SPI pin definitions pour ecran epaper
GxIO_Class io(SPI, /*CS=5*/ 0, /*DC=*/ 13, /*RST=*/ 25); // arbitrary selection of 17, 16  //SS remplacé par 0
GxEPD_Class display(io, /*RST=*/ 25, /*BUSY=*/ 4); // arbitrary selection of (16), 4

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

// variable fichier pour SD
String datachain = "";                   // chaine de donnée texte de mesure   

//déclaration pour la gestion de fichier sur la carte SD
String filename, filename_temp, str_index, filetrans;
int ind;                  // index vérification de fichier


//déclaration pour capteur de pression
MS5837 sensor;
float  wat_pressure, wat_temp, wat_depth, alt;


void setup() {
  Serial.begin(115200);                        // communication PC
  Wire.begin();                                // for I2C communication
  display.init();                              // enable display Epaper
  delay(500); //Take some time to open up the Serial Monitor and enable all things  

  affichageintro();
  delay (1000);
  
  // Test carte SD
  Serial.print("Initializing SD card...");   
    if (!SD.begin(5)) {                      // // see if the card is present and can be initialized, ajouter ici chipSelect ou 5 pour la pin 5 par default
    Serial.println("Card failed, or not present");
    errormessage();
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");   

  //création nom de fichier :
      // lecture de l'horloge rtc pour création de nom de fichier
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

      // création du nom de fichier et écriture sur la carte SD en automatisant la vérification de fichiers existant.
      filename =""; filename +="/"; filename += datenum;
      filename_temp = filename + String(0)+ String(0) + ".csv";           //1er fichier du jour
      int ind=0;

      while(SD.exists(filename_temp)){                             // vérifie l'existantce de fichier dans un boucle en indexant le numéro de fichier
        ind++;       
        if(ind<10){
          str_index=String(0)+String(ind);
        }
        else{
          str_index=String(ind);
        }
        filename_temp = filename + str_index + ".csv";
      }

      filename = filename_temp;   // création du nom de fichier final  
  // fin de création de fichier


  // 1ere ligne du fichier
  datachain += "Date"; datachain += " ; "; datachain += "Time" ; datachain += " ; ";
  datachain += "Profondeur"; datachain += " ; ";   datachain += "Temperature "; datachain += " ; ";
  

  // afficher la datachain sur le port serie
      Serial.println(" Format de la chaine enregistrée : ");
      Serial.println(datachain);

  // enregistrement de la datachain sur la carte SD
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

  // Initialize pressure sensor
  while (!sensor.init()) {                                            // Returns true if initialization was successful
    Serial.println("Init failed!");                                     // We can't continue with the rest of the program unless we can initialize the sensor
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(5000);
  }
  sensor.setModel(MS5837::MS5837_30BA);
  sensor.setFluidDensity(1029); // kg/m^3 (1029 for seawater, 997 for freshwater)





  delay(500);
  
}

void loop() {

      // lecture de l'horloge rtc
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

      // Make measure 
      mesure_pressure();
      
      // Création de la chaine à enregistrer sur la carte SD
      String datachain ="";
      datachain += year; datachain += "/";datachain += month; datachain += "/";datachain += date; datachain += " ; ";
      datachain += hour; datachain += ":";datachain += minute; datachain += ":"; datachain += second; datachain += " ; ";
      datachain += wat_depth; datachain += " ; "; datachain += wat_temp; datachain += " ; "; 
      
      // enregistrement sur la carte SD
        File dataFile = SD.open(filename, FILE_APPEND);
        if (dataFile) {                                        // if the file is available, write to it:
          dataFile.println(datachain);
          dataFile.close();
        }
        else {                                                 // if the file isn't open, pop up an error:
          Serial.println("error opening file");  
          errormessage();
          delay(3000);  
        }


      // Affichage ecran des datas
      Serial.println("Datachain : "); Serial.println(datachain); //affichage de la chaine complete sur le port serie

      // affichage ecran des data 
      display.setRotation(3);
      display.fillScreen(GxEPD_WHITE);
      
      display.setTextColor(GxEPD_BLACK);
      display.setFont(f3);
      display.setCursor(10, 40); display.println(wat_depth);
      display.setCursor(172,40); display.println(wat_temp);   
      display.setFont(f2);
      display.setCursor(40,70);display.print("m");
      display.setCursor(200,70);display.print("deg C");
    
      //cadre
      display.fillRect(147, 0, 2, 90, GxEPD_BLACK);
      display.fillRect(0, 90, 296, 2, GxEPD_BLACK); 
      
      //Last update
      display.setFont(f1);
      display.setCursor(0,108); display.print(date);display.print("/");display.print(month);display.print("/");display.print(year);  //date
      display.setCursor(123,108);display.print(hour);display.print("h");display.print(minute);                                       //heure
//      display.setCursor(205,108);display.print("Bat:");display.print(soc);display.print("%");                                        //batterie
//      display.setCursor(0, 125); display.print("Lat:");display.print(gps.location.lat(), 5);                                                     //lattitude
//      display.setCursor(148,125); display.print("Lng:");display.print(gps.location.lng(), 4);                                                   //longitude
     
      display.update();


   delay (5000);
}


void mesure_pressure(){
  // Update pressure and temperature readings
  sensor.read();

  wat_pressure = sensor.pressure();
  wat_temp=sensor.temperature();
  wat_depth=sensor.depth();
  alt=sensor.altitude();

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
}

void affichageintro(){                     // texte intro à l'allumage
  display.setRotation(3);
  display.fillScreen(GxEPD_WHITE);
  
  //titre
  display.setTextColor(GxEPD_BLACK);
  display.setFont(f4);
  display.setCursor(25, 55  );
  display.println("MESURE ");

 
  display.update();
}


void errormessage(){                     // texte intro à l'allumage

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
