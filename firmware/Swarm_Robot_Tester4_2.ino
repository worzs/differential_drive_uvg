/*
***************************************************************
Titulo: SwarmBot_Final.ino
Autores: William Orozco,Otto Wantland y Vidal Villegas
Funcion: Programa para el manejo del teensy.
***************************************************************
*/


/*
***************************************************************
                    Variables
***************************************************************
*/



#define BUFFER_SIZE 1024                                              // Tamanio de nuestro Buffer
/*
********************************************************************************************************
*********************************************ESP-8266***************************************************
********************************************************************************************************
*/
#define SSID  "dd-wrt"                                                // SSID de nuestra Red
#define PASS  ""                                                      // Contrasenia
#define PORT  "8080"                                                    // Definimos nuestro puerto a usar
#define TIME_OUT_SERIAL_AVAILABLE 200000

#define DESBORDES_TO_HCSR04 6

//Conexion
char buffer[BUFFER_SIZE];                                             // Buffer de los datos transmitidos
int vez = -1;
int data[] = {0,0,0,0,0,0,0,0};                                       // Array de los datos de nuestros distintos sensores
int options[] = {-1,-1};                                              // Maneja las posibles acciones dentro de nuestra interrupcion
String input;                                                         // Guarda nuestros datos de entrada del serial
int largo;                                                            // Guarda el length de nuestros datos de salida
int max_v = -1;                                                       // Variables para el manejo de las ordenes
int max_i = 0;
char OKrn[] = "OK\r\n";                                               // Respuesta esperada tras cada instruccion al ESP-8266
int finished = -1;
char character;
unsigned long t_esp_resp, tPrev_serialAvailable, t_serialAvailable;
unsigned long timeOut_serialAvailable;
bool found;
int len_esp_resp;
int index_resp;
String message;
int lengo_data_send;

volatile bool flagRecepcionSerial = false;                    //Bandera de recepcion serial 
boolean flagDataSend = false; 
volatile bool flagDriverMotores = false;
volatile byte contador_desbordes_to_hcsr04 = 0;
/*
********************************************************************************************************
******************************************Falda Sensores************************************************
********************************************************************************************************
*/
#define TRIG_PIN_0 6
#define TRIG_PIN_1 5
#define TRIG_PIN_2 4
#define ECHO_PIN_0 7
#define TIME_OUT_PULSE 50000

const byte SENSOR_HCSR04[] = {0,1,2,3,4,5}; //orden en que se hace el barrido de los sensores ultrasonicos

unsigned int duration_0, duration_1 , duration_2, duration_3, duration_4, duration_5; 
unsigned int distance0, distance1, distance2, distance3, distance4, distance5;   


volatile byte index_sensor_hcsr04;
volatile bool flagFallingEcho, flagSerialSend, flagCambioCodigoUltrasonico, flagIMU;
volatile unsigned long t_actual, t_anterior;
IntervalTimer echoTimeOut, timerEncoder; //objetos timer



//******************************************************************************************************
//********************************************Motores***************************************************       
//******************************************************************************************************
int velL = 255; 
int velR = 255;
bool dirL = 1;
bool dirR = 1;
const byte fr = 80;                                                   // A pin -> the interrupt pin 0
const byte encoder0_pinA = 2;                                          // A pin -> the interrupt pin 0
const byte encoder0_pinB = 3;                                          // B pin -> the digital pin 4
const byte encoder1_pinA = 11;                                         // A pin -> the interrupt pin 0
const byte encoder1_pinB = 12;                                         // B pin -> the digital pin 4
const byte driver_1A = 22;                                             // B pin -> the digital pin 4
const byte driver_1B = 23;                                             // B pin -> the digital pin 4
const byte driver_2A = 9;                                              // B pin -> the digital pin 4
const byte driver_2B = 10;                                             // B pin -> the digital pin 4
byte encoder0_pinALast_1,encoder0_pinALast_2;
volatile int duracion1,duracion2,velocidad1,velocidad2,p1,p2;        
bool Direction1,Direction2;       
bool flagE1, flagE2 = false;     
bool Lstate1, val1, Lstate2, val2;                           
volatile String direccion;

/*
***************************************************************
                    Funciones - Comunicacion
***************************************************************
*/

//Funcion para establecer un timeout para las respuestas del ESP8266
byte waitForEspResponse(int timeout, char* term=OKrn) 
{
  t_esp_resp=millis();
  found=false;
  index_resp=0;
  len_esp_resp=strlen(term);
  while(millis()<t_esp_resp+timeout)                                           // Espera el tiempo de tiemout o a recibir Okrn 
  {
    if(Serial1.available()) 
    {
      buffer[index_resp++]=Serial1.read();                                     // Si esta recibiendo algo lo cargamos a nuestro Buffer
      if(index_resp>=len_esp_resp)                                                      // Si ya llegamos al tamanio de nuestra respuesta, revisamos
      {
        if(strncmp(buffer+index_resp-len_esp_resp, term, len_esp_resp)==0)                       // Comparamos lo recibido con el termino 
        {
          found=true;
          break;
        }
      }
    }
  }
  buffer[index_resp]=0;
  return found;
}

//Funcion para establecer nuestra conexion WIFI a traves del ESP 8266
void setupWiFi() 
{
  Serial1.println("AT+RST");                                          // Reseteamos el modulo
  waitForEspResponse(3000);
  delay(500);
  Serial1.println("AT+CWLAP");                                        // Listamos las redes disponibles para nuestro modulo
  waitForEspResponse(5000);
  Serial1.print("AT+CWJAP=\"");                                       // Nos unimos a la red que queremos
  Serial1.print(SSID);
  Serial1.print("\",\"");
  Serial1.print(PASS);
  Serial1.println("\"");
  waitForEspResponse(20000);
}

//Funcion para establecer nuestro robot como servidor para el modo uno a uno
void setupTCP()
{  
  Serial1.println("AT+CWMODE=1");                                     // Indicamos que queremos el modo 1, direccion estatica
  waitForEspResponse(1000); 
  Serial1.println("AT+CWMODE?");                                      // Indicamos que queremos el modo 1, direccion estatica
  waitForEspResponse(1000); 
  Serial1.println("AT+CIPMUX=1");                                     // Establecemos que queremos una conexion unica
  waitForEspResponse(1000);
  Serial1.print("AT+CIPSERVER=1,");                                   // Establecemos el servidor TCP-IP
  Serial1.println(PORT);
  waitForEspResponse(1000);
}

//Funcion para el envio de datos desde nuestro Teensy hasta nuestra sesion de MAtlab
void DataSend(int data_data_send[], int largo_data_send)
{
  message="";
  digitalWrite(13,0);
  delayMicroseconds(100);
  for (int i = 0; i < largo_data_send; i++)                                     // Colocamos cada byte para el envio
  {  
    message.concat(data_data_send[i]);                                          // Creamos nuestro String de envio con nuestros datos
    message.concat(",");
  }
  lengo_data_send = message.length();                                       // Dictamos la cantidad de bytes a enviar
  Serial1.print("AT+CIPSEND=0,");                                     // Enviamos nuestra String de datos
  Serial1.println(lengo_data_send);
  waitForEspResponse(50);
  Serial1.println(message);
  waitForEspResponse(50);
  //Serial1.println("AT+CIPCLOSE=0");                                   // Cerramos la comunicacion TCP-IP
  //waitForEspResponse(50); 
}

/*
***************************************************************
                    Funciones - Falda de sensores
***************************************************************
*/

//Funcion que permite la toma de datos de nuestra falda de sensores ultrasonicos
void codigoUltrasonico()
{
  if (flagCambioCodigoUltrasonico){
   flagCambioCodigoUltrasonico = false;
  
   if (SENSOR_HCSR04[index_sensor_hcsr04]==0){
        if (flagFallingEcho)
          {flagFallingEcho=false;
          duration_0 = t_actual-t_anterior;
          if (duration_0>TIME_OUT_PULSE){
            duration_0 = TIME_OUT_PULSE;}}   
        else 
         {duration_0 = TIME_OUT_PULSE;}
        distance0 = (duration_0/60) ;
        digitalWrite(TRIG_PIN_0, HIGH); 
        digitalWrite(TRIG_PIN_1, LOW); 
        digitalWrite(TRIG_PIN_2, LOW);
      }
    else if (SENSOR_HCSR04[index_sensor_hcsr04]==1){
        if (flagFallingEcho){
          flagFallingEcho=false;
          duration_1 = t_actual-t_anterior;
          if (duration_1>TIME_OUT_PULSE){
            duration_1 = TIME_OUT_PULSE;}}
        else 
         {duration_1 = TIME_OUT_PULSE;}
        distance1 = (duration_1/60) ;
        digitalWrite(TRIG_PIN_0, LOW); 
        digitalWrite(TRIG_PIN_1, HIGH); 
        digitalWrite(TRIG_PIN_2, LOW);


        //flagIMU = imu.getMotion6Counts(&ax, &ay, &az, &gx, &gy, &gz);
      } 
    else if (SENSOR_HCSR04[index_sensor_hcsr04]==2){
        if (flagFallingEcho){
          flagFallingEcho=false;
          duration_2 = t_actual-t_anterior;
          if (duration_2>TIME_OUT_PULSE){
            duration_2 = TIME_OUT_PULSE;}}   
        else 
         {duration_2 = TIME_OUT_PULSE;}
        distance2 = (duration_2/60) ;
        digitalWrite(TRIG_PIN_0, HIGH); 
        digitalWrite(TRIG_PIN_1, HIGH); 
        digitalWrite(TRIG_PIN_2, LOW);

      } 
    else if (SENSOR_HCSR04[index_sensor_hcsr04]==3){
        if (flagFallingEcho)
          {flagFallingEcho=false;
          duration_3 = t_actual-t_anterior;
          if (duration_3>TIME_OUT_PULSE){
            duration_3 = TIME_OUT_PULSE;}}   
        else 
         {duration_3 = TIME_OUT_PULSE;}
        distance3 = (duration_3/60) ;
        digitalWrite(TRIG_PIN_0, LOW); 
        digitalWrite(TRIG_PIN_1, LOW); 
        digitalWrite(TRIG_PIN_2, HIGH);


        //flagIMU = imu.getMotion6Counts(&ax, &ay, &az, &gx, &gy, &gz);
      } 
    else if (SENSOR_HCSR04[index_sensor_hcsr04]==4){
        if (flagFallingEcho)
          {flagFallingEcho=false;
          duration_4 = t_actual-t_anterior;
          if (duration_4>TIME_OUT_PULSE){
            duration_4 = TIME_OUT_PULSE;}}
        else 
         {duration_4 = TIME_OUT_PULSE;}
        distance4 = (duration_4/60) ;
        digitalWrite(TRIG_PIN_0, HIGH); 
        digitalWrite(TRIG_PIN_1, LOW); 
        digitalWrite(TRIG_PIN_2, HIGH);
      }
    else if (SENSOR_HCSR04[index_sensor_hcsr04]==5){
        if (flagFallingEcho)
          {flagFallingEcho=false;
          duration_5 = t_actual-t_anterior;
          if (duration_5>TIME_OUT_PULSE){
            duration_5 = TIME_OUT_PULSE;}}   
        else 
         {duration_5 = TIME_OUT_PULSE;}
        distance5 = (duration_5/60) ;
        digitalWrite(TRIG_PIN_0, LOW); 
        digitalWrite(TRIG_PIN_1, LOW); 
        digitalWrite(TRIG_PIN_2, LOW);

        //flagIMU = imu.getMotion6Counts(&ax, &ay, &az, &gx, &gy, &gz); 
      }
    
 }
  noInterrupts();
  data[0] = distance0;                                                // Asignamos nuestros datos al array para envio
  data[1] = distance1;
  data[2] = distance2;
  data[3] = distance3;
  data[4] = distance4;
  data[5] = distance5;
  data[6] = duracion1;
  data[7] = duracion2;
  interrupts();
  } 

void changeEchoIsr(){
    if (digitalRead(ECHO_PIN_0)==HIGH){//rising
        t_anterior=micros();
      }
    else{
        flagFallingEcho=true;
        t_actual=micros();
      }
  }

void timeOutEchoIsr(){
  index_sensor_hcsr04=index_sensor_hcsr04+1;
  if (index_sensor_hcsr04>5)
  {index_sensor_hcsr04=0;}
  flagCambioCodigoUltrasonico = true;
  contador_desbordes_to_hcsr04 = contador_desbordes_to_hcsr04+1;
  if (contador_desbordes_to_hcsr04>=DESBORDES_TO_HCSR04){
      contador_desbordes_to_hcsr04=0;
      //flagDataSend=true;
    }
  
  }
/*
***************************************************************
                    Funciones - Motores
***************************************************************
*/

//Funcion para dictar direccion y velocidad a nuestros motores.
/*void drivers(int direccion, int p1, int p2)
{
  switch (direccion) {
    case 1 :                                                          //"adelante L"
      //analogWrite(driver_1B,p1);
      //analogWrite(driver_1A,255);
      //analogWrite(driver_2A,p2);
      //analogWrite(driver_2B, 255);      
      analogWrite(driver_1B,p1);
      analogWrite(driver_1A,255);
      break;
    case 2 :                                                          //"atras L"
      //analogWrite(driver_1B,255);
      //analogWrite(driver_1A,p1);
      //analogWrite(driver_2A,255);
      //analogWrite(driver_2B, p2);
      analogWrite(driver_1B,255);
      analogWrite(driver_1A,p1);
    break;
    case 3 :                                                          //"Adelante R" 
      //analogWrite(driver_1B,p1);
      //analogWrite(driver_1A,255);
      //analogWrite(driver_2A,255);
      //analogWrite(driver_2B, p2);
      analogWrite(driver_2A,p1);
      analogWrite(driver_2B, 255);
      
    break;
    case 4:                                                           //"Atras R"
      //analogWrite(driver_1B,255);
      //analogWrite(driver_1A,p1);
      //analogWrite(driver_2A,p2);
      //analogWrite(driver_2B, 255);
      analogWrite(driver_2A, 255); 
      analogWrite(driver_2B, p1);
    break;
    
  }
}*/

void drivers(bool dL, int p1, bool dR, int p2)
{
  //noInterrupts();
  if (dL){
      analogWrite(driver_1B,p1);
      analogWrite(driver_1A,255);
    }
  else{
      analogWrite(driver_1B,255);
      analogWrite(driver_1A,p1);
    }
  if (dR){
      analogWrite(driver_2A,p2);
      analogWrite(driver_2B, 255);
    }
  else{
      analogWrite(driver_2A, 255); 
      analogWrite(driver_2B, p2);
    }
  //interrupts();  
  }


//Funcion para la asignacion de los valores de nuestros encoders en el array para envio
void countreset()
{
    data[6] = duracion1;
    data[7] = duracion2;
    //duracion1 = 0;
    //duracion2 = 0;
    //flagDriverMotores = true;
}

//Funciones de manejo de la velocidad de una de las ruedas
void wheelSpeed2()
{
  //flagE2 = true;
  Lstate2 = digitalRead(encoder1_pinA);
  if((encoder0_pinALast_2 == LOW) && Lstate2==HIGH)
  {
    val2 = digitalRead(encoder1_pinB);
    if(val2 == LOW && Direction2)
    {
      Direction2 = false; //Reverse
    }
    else if(val2 == HIGH && !Direction2)
    {
      Direction2 = true;  //Forward
    }
  }
  encoder0_pinALast_2 = Lstate2;
  //Direction1 = false;
  if(!Direction2)  duracion2 ++;
  else  duracion2 --;
}


void wheelSpeed1()
{
  //flagE1 = true;
  Lstate1 = digitalRead(encoder0_pinA);
  if((encoder0_pinALast_1 == LOW) && Lstate1==HIGH)
  {
    val1 = digitalRead(encoder0_pinB);
    if(val1 == LOW && Direction1)
    {
      Direction1 = false;                                               //Reverse
    }
    else if(val1 == HIGH && !Direction1)
    {
      Direction1 = true;                                                //Forward
    }
  }
  encoder0_pinALast_1 = Lstate1;
  if(!Direction1)  duracion1 ++;
  else  duracion1--;
}


void wheelSpeed1C(){
  noInterrupts();
  Lstate1 = digitalRead(encoder0_pinA);
  if((encoder0_pinALast_1 == LOW) && Lstate1==HIGH)
  {
    val1 = digitalRead(encoder0_pinB);
    if(val1 == LOW && Direction1)
    {
      Direction1 = false;                                               //Reverse
    }
    else if(val1 == HIGH && !Direction1)
    {
      Direction1 = true;                                                //Forward
    }
  }
  encoder0_pinALast_1 = Lstate1;
  if(!Direction1)  duracion1 ++;
  else  duracion1--;
  interrupts(); 
}

void wheelSpeed2C()
{
  noInterrupts();
  Lstate2 = digitalRead(encoder1_pinA);
  if((encoder0_pinALast_2 == LOW) && Lstate2==HIGH)
  {
    val2 = digitalRead(encoder1_pinB);
    if(val2 == LOW && Direction2)
    {
      Direction2 = false; //Reverse
    }
    else if(val2 == HIGH && !Direction2)
    {
      Direction2 = true;  //Forward
    }
  }
  encoder0_pinALast_2 = Lstate2;
  //Direction1 = false;
  if(!Direction2)  duracion2 ++;
  else  duracion2 --;
  
  interrupts();
}



/*
***************************************************************
                      Setup
***************************************************************
*/

void setup() 
{
  //El ESP8266 se comunica a 115200 a traves del serial.
  Serial1.begin(115200);  // Serial 1 del Teensy
  //Serial.begin(9600);    // Comunicacion con la computadora
  pinMode(13,OUTPUT);
  digitalWrite(13,0);
  //Esperamos un momento e inicializamos nuestro ESP8266
  delay(1000);
  
  //Serial.println("begin.");  
  setupWiFi();
  setupTCP();
  
  // Mostramos la direccion IP asignada
  //Serial.print("device ip addr: ");
  Serial1.println("AT+CIFSR");
  waitForEspResponse(5000);

  //Definimos los pines de nuestra falda de sensores
  pinMode(TRIG_PIN_0, OUTPUT);
  pinMode(TRIG_PIN_1, OUTPUT);
  pinMode(TRIG_PIN_2, OUTPUT);
  pinMode(ECHO_PIN_0, INPUT);

  //falda de sensores
  duration_0=0;
  duration_1=0;
  duration_2=0;
  duration_3=0;
  duration_4=0;
  duration_5=0;
  
  distance0=0;
  distance1=0;
  distance2=0;
  distance3=0;
  distance4=0;
  distance5=0;

  t_actual=0;
  t_anterior=0;
  flagFallingEcho = false;
  flagSerialSend = false;
  flagCambioCodigoUltrasonico = false;
  flagIMU=false;
  index_sensor_hcsr04=0;
  
  digitalWrite(TRIG_PIN_0, HIGH); 
  digitalWrite(TRIG_PIN_1, HIGH); 
  digitalWrite(TRIG_PIN_2, HIGH);  
  delayMicroseconds(20);
  attachInterrupt(ECHO_PIN_0, changeEchoIsr, CHANGE);
  echoTimeOut.begin(timeOutEchoIsr, TIME_OUT_PULSE); //timer para la espera maxima de cada pulso
  
  //recepcion serial
  flagRecepcionSerial=false;

  //PWM
   analogWriteFrequency(driver_1A , fr);
   analogWriteFrequency(driver_1B , fr);
   analogWriteFrequency(driver_2A , fr);
   analogWriteFrequency(driver_2B , fr);

  //Encoder
  timerEncoder.begin(countreset, 1000000); 
  //Timer1.initialize(1000000);
  //Timer1.attachInterrupt(countreset);
  Direction1 = true;//default -> Forward  
  Direction2 = true;
  pinMode(encoder0_pinA,INPUT);
  pinMode(encoder0_pinB,INPUT); 
  pinMode(encoder1_pinA,INPUT);
  pinMode(encoder1_pinB,INPUT); 
  
  attachInterrupt(encoder0_pinA, wheelSpeed1, CHANGE);  
  attachInterrupt(encoder1_pinA, wheelSpeed2, CHANGE);

  //valores menores significan prioridad mayor
  NVIC_SET_PRIORITY(IRQ_PORTD, 32); //el pin 7 de echo esta en puertoD, 2
  NVIC_SET_PRIORITY(IRQ_PORTC, 48); //los pines de lectura de encoder estan en puerto C, 1 2 3 y 4. 
  
  drivers(1,255,1,255); //se inicializan los motores
  //digitalWrite(13,1);
}

/*
***************************************************************
                    Loop
***************************************************************
*/
void loop() {
  digitalWrite(13,1);
  
  /****Toma las medidas de la falda de sensores****/
  codigoUltrasonico();

  /*****Funciones de encoders*****/
  if (flagE1 == true){
   // flagE1 = false;
   // wheelSpeed1C();
  }

  if (flagE2 == true){
    //flagE2 = false;
    //wheelSpeed2C();
  }
  
  /*****Recepcion de datos*****/
  recepcionSerial();

  /*****Manda la orden a los motores****/
  if (flagDriverMotores){
    flagDriverMotores=false;
    drivers(dirL,velL,dirR,velR);
  }
  
  /*****Envio de datos*****/
  if (flagDataSend){
    flagDataSend=false;
    largo = sizeof(data);
    largo = largo/sizeof(data[0]); 
    DataSend(data, largo); 
  }
}





/**************************************************************
 * Interrupcion por entrada de comunicacion con el modulo WI-FI
**************************************************************/
void serialEvent1(){ 
  flagRecepcionSerial=true;
}


//funcion que recibe la informacion del puerto serial
void recepcionSerial(){
  if (flagRecepcionSerial){
    flagRecepcionSerial=false;
    digitalWrite(13,0);
    finished = -1;
    tPrev_serialAvailable=millis();
    while (Serial1.available())                                       // Tomamos todos los datos de entrada
    {
      character = (char)Serial1.read();
      input += character; 
      finished = input.lastIndexOf("CLOSED"); 
      t_serialAvailable=millis()-tPrev_serialAvailable;
      /*if(t_serialAvailable>TIME_OUT_SERIAL_AVAILABLE){
          //finished=-1;
          Serial1.flush();
          Serial1.clear();
          buffer[0]='\0';
          break;
        }*/
       //if (finished>-1){
        //break;
        //}
    }
    if(finished > -1)                                                 // Tomamos solamente el envio exitoso de datos
    {
      //flagDataSend=true;
      
      options[0] = input.lastIndexOf("CONNECT");                      // Revisamos la instruccion enviada
      options[1] = input.lastIndexOf("Motores");
      max_v=-1;
      max_i = 0;
      for ( int i = 0; i < sizeof(options)/sizeof(options[0]); i++ )
      {
        if ( options[i] > max_v )
        {
          max_v = options[i];
          max_i = i;
        }
        if (i>100){
            max_i = 2;
            break;
          }
      }
      
      switch (max_i) 
      {
        case 0 :                                                        // Pedida de datos de la falda de sensores
          //largo = sizeof(data);
          //largo = largo/sizeof(data[0]); 
          //DataSend(data, largo);
          flagDataSend=true;
          break;
        case 1 :                                                        // Asignamos valores a nuestros motores
          noInterrupts();
          dirL     =  (input.charAt(options[1] + 7) - 48);
          velL  =  (input.charAt(options[1] + 8) - 48)*100;
          velL  += (input.charAt(options[1] + 9) - 48)*10;
          velL  += (input.charAt(options[1] + 10) - 48);
      
          dirR     =  (input.charAt(options[1] + 11) - 48);
          velR  =  (input.charAt(options[1] + 12) - 48)*100;
          velR  += (input.charAt(options[1] + 13) - 48)*10;
          velR  += (input.charAt(options[1] + 14) - 48);
          interrupts();

          
          //drivers(Order,movement,movement1);  
          flagDriverMotores = true;
          flagDataSend=true;
          
          break;
        case 2:
          flagDataSend=true;
          break;
      }
    }
  
    //Serial1.flush();
    Serial1.clear();
    }
  }

