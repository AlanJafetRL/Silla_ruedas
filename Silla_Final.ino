//Librería y definición de puertos para bt
#include <SoftwareSerial.h>
SoftwareSerial BT(51,53);  //RX, TX
// Librerias I2C para controlar el mpu6050
// la libreria MPU6050.h necesita I2Cdev.h, I2Cdev.h necesita Wire.h
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
// La dirección del MPU6050 puede ser 0x68 o 0x69, dependiendo 
// del estado de AD0. Si no se especifica, 0x68 estará implicito
MPU6050 sensor;
//Definiciones
#define SOUND 0.01723 //Define velocidad del sonido
#define sensibilidad 0.100  //Sensibilidad en mV/A para sensor de 20A
#define n_muestras 250  //Numero de muestras para obtener promedio de corriente
//Tiempos (ms)
#define contraer_rst 2000 //Tiempo para contraer los actuadores luego de activar el rst de emergencia
#define periodo_impr 250  //Periodo en ms a los que se imprimirán los datos
//---Definición de los pines a utilizar---//
//--Sensores
#define s1t 22
#define s1e 24
#define s2t 26  
#define s2e 28
#define s3t 30
#define s3e 32
#define s4t 31
#define s4e 33
#define s5t 27
#define s5e 29  
#define s6t 23
#define s6e 25
#define c1 0 //Corriente act1
#define c2 14  //Corriente act2
#define vb 8 //Voltaje bateria
//--Actuadores
#define act1pwm 5 //Pin enable
#define act1R 6 //Pines pwm
#define act1L 7
#define act2R 9
#define act2L 8
#define act2pwm 10  //Pin enable
#define vel 1023
#define velmin 500
//--Calibracion angulos
#define subidaideal -7
#define bajadaideal 7
//--Botones
#define btn_up 3 //int0
#define btn_dw 2  //int4
#define btn_rst 19 //int5 
//--Anti rebotes
#define rango_rebote 200  //Rango en el cual puede existir el rebote en la lectura de rst
#define doble_pulso 800 //El doble pulso puede darse entre 800-200=600ms 
//--Espera ciclo de bajada
#define rango_espera 5000 //5 segundos de espera para poner la silla en posición de bajada
//--Buzzer
#define buzz 13
#define NOTA_C4  262  //Error
#define NOTA_E5  659  //Espera bajada
#define NOTA_G5  784  //1er tono correcto
#define NOTA_C6  1047 //2do tono correcto y sistema de control
//Calibración de distancias
#define sens_suelo 5.7  //Sensor de las ruedas al suelo (s5) 
#define sens_suelo_s6 6.3 //Sensor de las ruedas al suelo (s6) 
#define agregado 0.5  //Agregado para sensor de las ruedas al suelo 
#define sens_banq 15.5  //Distancia entre el sensor trasero de la carcasa y la banqueta (s1)
#define sens_banq_s2 20.0  //Distancia entre el sensor trasero de la carcasa y la banqueta (s2)
#define agregado_banq 15.0 //Agregado para sensor trasero de la carcasa a la banqueta 
#define tope 110.0 //Límite de lectura de los sensores traseros
#define mod_suelo_s3 10.0  //Distancia entre el sensor inferior (3) de la carcasa y el suelo o banqueta  
#define mod_suelo_s4 8.5  //Distancia entre el sensor inferior (4) de la carcasa y el suelo o banqueta 
#define agregado_suelo 1.0 //Agregado para sensores inferiores de la carcasa al suelo (s3 y s4)
#define dif_s2_s3 4.5 //Diferencia vertical entre s2 y s3
#define dist_ruedas 11.0 //Distancia vertical inicial de la base de las ruedas
//Para imprimir en consola
#define opcion 2  //0->No imprime nada, 1-> Imprime en Serial, 2-> Imprime en BT
//Calibración de sensores
#define rangoAct 0.20 
//Sensor de corriente detecta rango -0.03 a  0.02. El actuador funciona con mínimo 1.2A aprox
//Por lo tanto el rango cuando el actuador ya está desactivado puede estar entre +-0.8 
//---Termina definicion de pines---//

//--------------------------------------------------

//------Declaración de variables globales------//
int s1;  //Variable sensor 1
int s21, s22, s23; //Variables para casos de medición del sensor 2
int s31, s32, s33; //Variables para casos de medición del sensor 3
int s4;
int s51, s52;
int s61, s62;
int a1, a2; //Variables para indicar activación de actuadores
int b0=0; //Variable para buzzer
int silencio=0; //Variable para silencios entre notas ciclicas
float pulso_btn_up, pulso_btn_dw, pulso_btn_rst;  //Variable para identificar que ya se pulsó subida o bajada
float distancia_u, distancia_d; //Variables para guardar temporalmente las distancias obtenidas de los sensores
float distancia_altura, dist_banqueta; //Variable para guardar la altura del modulo al suelo
float ds2, ds3, ds4;
unsigned long tiempo_impr=0;  //Tiempo para comparación de periodos con millis() (reemplazo de delay)
float cA1,cA2,vBat,nBat; //Variables para realizar comparaciones e impresión de valores de corriente, voltaje y nivel de batería
int ax, ay, az; //Variables para acelerómetro
float accel_ang_x=0, accel_ang_y=0; //Variales para guardar ángulos con acelerometro
float cont_pruebas=0;
//Variables de Strings
int impr_encabezado;
int tam_banq;
int e_actual;
int sens;
int c_act1, c_act2;
int ang_xy;
int nivel_bat;
int seq;
float *ptr_datos[7];  //Puntero para n_prueb, tamaño banqueta, lectura sensor, corriente1 y 2, ángulos XY y batería
int op; //Variable para impresión 1 o cada 250ms
float n_pulsos;
unsigned long tiempo_btn=0; //Variable para guardar tiempo en millis, btn_rst
unsigned long tiempo_bajada=0;  //Variable para guardar tiempo en millis, tiempo espera para bajar
unsigned long tiempo_tono=0;

//------Termina declaración de variables globales------//

//--------------------------------------------------

//------Definición de los estados------//
typedef enum { //Define las posibles constantes para los estados
  e0=0, //Estado de reposo (fuera de cualquier secuencia)
  up_e0=1, up_e1=2, up_e2=3, up_e3=4, up_e4=5, up_e5=6, up_e6=7, up_e61=19, up_e7=8, up_e81=21, up_e8=9, up_e9=10, //Estados para secuencia de subir
  dw_e0=11, dw_e1=12, dw_e12=23, dw_e2=13, dw_e3=14, dw_e31=20, dw_e4=15,  //Estados para secuencia de bajar
  rst_act=16, rst_act2=22, rst_fin=17  //Estado para regresar en caso de rst de emergencia
} estado;

estado edo_actual;  //Para controlar sincrónicamente los cambios de estado
estado aux_cont;    //Para controlar lo que muestra la consola
//------Termina definición de los estados------//

//--------------------------------------------------

//------Inicia definición de los Strings------//
//---Strings de encabezado de la tabla
String encabezado_array[11]={"\n\tNo. Prueba", "Tamaño banqueta (cm)", "Estado actual", "Lectura sensor (cm)", "Corriente act1 (A)", "Corriente act2 (A)", "Inclinación (X°)", "Inclinación (Y°)", "Batería (%)", "Nota de secuencia (up,dw,rst)"};
//---Strings de estado actual
String e_actual_array[23]={
  " - e0", 
  " - up_e0", 
  " - up_e1", 
  " - up_e2", 
  " - up_e3", 
  " - up_e4", 
  " - up_e5", 
  " - up_e6", 
  " - up_e7", 
  " - up_e8", 
  " - up_e9", 
  " - dw_e0", 
  " - dw_e1",
  " - dw_e2",
  " - dw_e3",
  " - dw_e4",
  " - rst_act",
  " - rst_fin",
  "",  //18
  " - up_e61", 
  " - dw_e31",
  " - up_e81",
  " - rst_act2"
};
//---Strings de distancia medida por sensor
String sens_array[23]={
  "", //0 -> e0
  "S2: ", //1 -> up_e0
  "S1: ", //2 -> up_e1
  "S2: ", //3 -> up_e2
  "S3: ", //4 -> up_e3
  "S3: ", //5 -> up_e4
  "S3: ", //6 -> up_e5
  "S5: ", //7 -> up_e6
  "S4: ", //8 -> up_e7
  "S6: ", //9 -> up_e8
  "", //10 -> up_e9
  "S6: ", //11 -> dw_e0
  "S6: ", //12 -> dw_e1
  "S5: ", //13 -> dw_e2
  "S4: ", //14 -> dw_e3
  "", //15 -> dw_e4
  "", //16 -> rst_act
  "", //17 -> rst_fin
  "", //18 -> dw_e4
  "S5: ", //19 -> up_e61
  "S4: ", //20 -> dw_e31
  "S6: ", //21 -> up_e81
  "" //22 -> rst_fin
};
//---Strings de nota de secuencia
String seq_array[23]={
  //Se utilizarán cada que exista un cambio de estado exitoso, después regresará a 0 para no imprimir nada cuando no se cambie de estado
  "", //0 -> e0
  "S2 detectó altura mayor a 15cm", //1 -> up_e0
  "S1 detectó altura menor a 30cm", //2 -> up_e1
  "S2 detectó que ya se supero la banqueta", //3 -> up_e2
  "S3 guarda distancia elevada", //4 -> up_e3
  "S3 detectó que la silla ya se elevó a la altura adecuada", //5 -> up_e4
  "S3 detecta que la parte trasera ya esta en la banqueta", //6 -> up_e5
  "S5 detecta que el actuador 1 ya se contrajo completamente", //7 -> up_e6
  "S4 detectó que ya está en la banqueta", //8 -> up_e7
  "S6 detecta que el actuador 2 ya se contrajo completamente", //9 -> up_e8
  "Termina ciclo de subida. Reiniciando...", //10 -> up_e9
  "S6 detectó distancia entre 15 y 30 cm", //11 -> dw_e0
  "S6 detectó que las ruedas delanteras están por tocar el suelo", //12 -> dw_e1
  "S5 detectó que las ruedas traseras ya pasaron la banqueta", //13 -> dw_e2
  "S4 detectó que el actuador 2 ya se contrajo completamente", //14 -> dw_e3
  "Termina ciclo de bajada. Reiniciando...", //15 -> dw_e4
  "Reiniciando...Contrayendo actuadores", //16 -> rst_act
  "***Fin***", //17 -> rst_fin
  //Casos de error
  "S2 detectó altura menor a 15cm. Reiniciando...", //18 -> up_e0
  "S1 detectó altura mayor a 30cm. Reiniciando...", //19 -> up_e1
  "S3 guarda distancia elevada. La altura es adecuada, salto de estado...", //20 -> up_e3
  "S6 detecto distancia fuera del rango de 15 y 30 cm", //21 -> dw_e0
  "Actuadores contraidos por completo" //22 -> rst_act2
};
//---Termina definición de los Strings---//

//--------------------------------------------------

//------Inician funciones------//
//---Inician funciones que inicializan---//
//---Funcion que inicializa todo
void funcsInicio () {
  Wire.begin();           //Iniciando I2C  
  sensor.initialize();    //Iniciando el sensor
  sensores();
  actuadores();
  botones();
  variablesCero();
  edo_actual=e0;
}

//---Funcion de inicio de conexion
void inicio() {
  if (opcion==1) {
    //Imprime por Serial
    Serial.println("\n---Conexión Serial---\nIniciando...");
    funcsInicio();
    if (sensor.testConnection()) {
      Serial.println("Sensor giroscopio iniciado correctamente");
    } else {
      Serial.println("Error al iniciar el sensor giroscopio");
    }
  } else if (opcion==2) {
    //Imprime por BT
    BT.println("\n---Conexión BT---\nIniciando...");
    funcsInicio();
    if (sensor.testConnection()) {
      BT.println("Sensor giroscopio iniciado correctamente");
    } else {
      BT.println("Error al iniciar el sensor giroscopio");
    }
  } else {
    //Cualquier otro caso no imprime nada
  }
}

//---Inicializa los sensores
void sensores() {
  //Sensor 1
  pinMode(s1t,OUTPUT); //Emisor trigger
  pinMode(s1e,INPUT);  //Receptor echo
  digitalWrite(s1t, LOW); //Emisor en bajo

  //Sensor 2
  pinMode(s2t,OUTPUT); //Tigger
  pinMode(s2e,INPUT);  //Echo
  digitalWrite(s1t, LOW); //Emisor en bajo

  //Sensor 3
  pinMode(s3t,OUTPUT); //Tigger
  pinMode(s3e,INPUT);  //Echo
  digitalWrite(s3t, LOW); //Emisor en bajo

  //Sensor 4
  pinMode(s4t,OUTPUT); //Tigger
  pinMode(s4e,INPUT);  //Echo
  digitalWrite(s4t, LOW); //Emisor en bajo

  //Sensor 5
  pinMode(s5t,OUTPUT); //Tigger
  pinMode(s5e,INPUT);  //Echo
  digitalWrite(s5t, LOW); //Emisor en bajo

  //Sensor 6
  pinMode(s6t,OUTPUT); //Tigger
  pinMode(s6e,INPUT);  //Echo
  digitalWrite(s6t, LOW); //Emisor en bajo
}

//---Inicializa los actuadores
void actuadores() {
  pinMode(act1R, OUTPUT); //Actuador 1
  pinMode(act1L, OUTPUT); //Actuador 1
  pinMode(act1pwm, OUTPUT); //Actuador 1

  pinMode(act2R, OUTPUT); //Actuador 2
  pinMode(act2L, OUTPUT); //Actuador 2
  pinMode(act2pwm, OUTPUT); //Actuador 2
}

//---Inicializa los botones
void botones() {
  attachInterrupt(digitalPinToInterrupt(btn_up), subir, FALLING); //Boton de subir detecta cambio de bajo a alto
  attachInterrupt(digitalPinToInterrupt(btn_dw), bajar, FALLING); //Boton de bajada detecta cambio de bajo a alto
  attachInterrupt(digitalPinToInterrupt(btn_rst), rst, FALLING); //Boton de rst detecta cambio de bajo a alto
}

//---Función que da valores iniciales a variables globales
void variablesCero() {
  s1=0;
  s21=0; s22=0; s23=0;
  s31=0; s32=0; s33=0;
  s4=0;
  s51=0; s52=0;
  s61=0; s62=0;
  a1=0; a2=0;
  b0=0;
  pulso_btn_up=0; pulso_btn_dw=0; pulso_btn_rst=0;
  edo_actual=e0;  //Inicia en e0 (fuera de cualquier secuencia)
  aux_cont=rst_fin;  //Inicia auxiliar en otro estado para identificar un cambio de estados inicial
  distancia_u=0; distancia_d=0; //Inicia distancias en 0
  distancia_altura=0; dist_banqueta=0;
  ds2=0; ds3=0; ds4=0;
  tam_banq=0;
  sens=0;
  c_act1=1; c_act2=1; ang_xy=1; nivel_bat=1;  //Siempre imprimirá corrientes, nivel de batería y ángulos
  seq=0;  //Inicia en 0 para no imprimir nada
  op=2; //Inicia en 2 para no imprimir nada
  n_pulsos=0;
  tiempo_btn=0; 
  tiempo_bajada=0;
  tiempo_tono=0;
  silencio=0;
}

//---Terminan funciones que inicializan---//
//---Inician funciones de operación---///

//---Función que controla actuador 1
void act1() {
  switch (a1) {
    case 0:
      //Detiene actuador
      digitalWrite(act1R, LOW);
      digitalWrite(act1L, LOW);
      analogWrite(act1pwm, 0);
      break;
    case 1:
      //Extiende actuador
      digitalWrite(act1R, LOW);
      digitalWrite(act1L, HIGH);
      analogWrite(act1pwm, vel);
      break;
    case 2:
      //Contrae actuador
      digitalWrite(act1R, HIGH);
      digitalWrite(act1L, LOW);
      analogWrite(act1pwm, vel);
      break;
    case 3:
      //Extiende actuador velocidad minima
      digitalWrite(act1R, LOW);
      digitalWrite(act1L, HIGH);
      analogWrite(act1pwm, velmin);
      break;
    case 4:
      //Contrae actuador velocidad minima 
      digitalWrite(act1R, HIGH);
      digitalWrite(act1L, LOW);
      analogWrite(act1pwm, velmin);
      break;
  }
}

//---Función que controla actuador 2
void act2() {
  switch (a2) {
    case 0:
      //Detiene actuador
      digitalWrite(act2R, LOW);
      digitalWrite(act2L, LOW);
      analogWrite(act2pwm, 0);
      break;
    case 1:
      //Extiende actuador
      digitalWrite(act2R, LOW);
      digitalWrite(act2L, HIGH);
      analogWrite(act2pwm, vel);
      break;
    case 2:
      //Contrae actuador
      digitalWrite(act2R, HIGH);
      digitalWrite(act2L, LOW);
      analogWrite(act2pwm, vel);
      break;
    case 3:
      //Extiende actuador
      digitalWrite(act2R, LOW);
      digitalWrite(act2L, HIGH);
      analogWrite(act2pwm, velmin);
      break;
    case 4:
      //Contrae actuador
      digitalWrite(act2R, HIGH);
      digitalWrite(act2L, LOW);
      analogWrite(act2pwm, velmin);
      break;
  }
}

//---Función interrupción del botón de subida ISR pin3
void subir() {
  if (edo_actual==e0) {
    edo_actual=up_e0; //Interrumpe estados y pasa directamente al up_e0
  }
  //En otro caso, no hace nada
}

//---Función interrupción del botón de bajada ISR pin2
void bajar() {
  if (edo_actual==e0) {
    pulso_btn_dw=1; //Cambia variable para indicar que se pulsó btn de bajar
    edo_actual=dw_e0; //Interrumpe estados y pasa directamente al dw_e0
  }
  //En otro caso, no hace nada
}

//---Función interrupción del botón de reset ISR pin19
void rst() {
  //Entra en cualquier estado y cualquier momento mientras se pulse el btn de reset 2 veces
  if(n_pulsos==0) {
    n_pulsos=1; //Cambia una variable
    //Serial.println("\nBtn: "+String(n_pulsos)+"\tEsperando segundo pulso");
    tiempo_btn=millis();  //Guarda tiempo donde se pulsó btn 1era vez 
  }
  if(n_pulsos==1 && millis()>tiempo_btn+rango_rebote) {  //Si ya pasaron más de 200ms desde que se presionó por primera vez
    if (millis()<tiempo_btn+doble_pulso) {  //Si aún no pasan más de 800ms
      n_pulsos=2; //Variable que indica que ya se pulsó 2 veces
      //Serial.println("Btn: "+String(n_pulsos)+"\tSe pulsó 2 veces. Reinicia...");
      edo_actual=rst_act; //Interrumpe estados y pasa directamente a reset
    } else {
      n_pulsos=0;
    }
  } else if (n_pulsos==1 && millis()<tiempo_btn+rango_rebote) { //Si no han pasado más de 200ms
    if (millis()<tiempo_btn+doble_pulso) {  //Si aún no pasan más de 800ms
      //Serial.println("Btn: "+String(n_pulsos)+"\tRebote");
    }
  } else if (n_pulsos==2 && millis()>tiempo_btn+doble_pulso+rango_rebote){  //Si ya pasó más de 1 segundo
    n_pulsos=0;  //Regresa a 0 para evitar que marque otro pulso después de 2do pulso exitoso    
  }
}

//---Función que define el trigger y echo del sensor indicado. Regresa la distancia medida
float leeUltra(int sensor) {
  //Variables para definir trigger y echo dependiendo del sensor que se indique
  int trigger=0, echo=0;
  //Variable para retornar el promedio de la distancia medida
  float prom_dist=0;
  //Switch case para indicar el sensor a utilizar
  switch (sensor) {
    case 1:
        trigger=s1t;
        echo=s1e;
      break;
    case 2:
        trigger=s2t;
        echo=s2e;
      break;
    case 3:
        trigger=s3t;
        echo=s3e;
      break;
    case 4:
        trigger=s4t;
        echo=s4e;
      break;
    case 5:
        trigger=s5t;
        echo=s5e;
      break;
    case 6:
        trigger=s6t;
        echo=s6e;
      break;
  }
  //Obtiene distancia medida de la funcion ultra
  prom_dist=ultra(trigger,echo);
  //Regresa la distancia en cm
  return prom_dist;
}

//---Función regresa la medición del sensor indicado
float ultra(int t, int e) {
  //Variables para sumar, guardar la duracion del echo y obtener distancia
  float  tiempo=0, distancia=0;
  //Emite sonido por 10us
  digitalWrite(t, HIGH);
  delayMicroseconds(10);
  //Apaga emisor y escucha la duracion del pulso
  digitalWrite(t,LOW);
  tiempo=pulseIn(e,HIGH);
  //Calcula distancia con la duracion
  distancia=tiempo*SOUND;
  return distancia;
}

//---Función que obtiene corriente del actuador especificado o bateria. Regresa corriente en Amper
float getCorriente(int AX) {
  float vSensor;
  float corriente=0;
  for (int i=0;i<n_muestras;i++) {
    vSensor=analogRead(AX)*(5.0/1023.0);  //Lectura del sensor*5v (máximo analógico)/1023(bits)
    corriente=corriente+(vSensor-2.5)/sensibilidad; //2.5 equivale a la mitad de 5v, es para definir negativos y positivos
  }
  corriente=corriente/n_muestras;
  return(corriente-0.01); //-0.01 para dar más precisión por lo que se calibró
}

//---Función que obtiene el ángulo ***Sistema de control***
void angulos() {
  //Lee las aceleraciones
  sensor.getAcceleration(&ax, &ay, &az);
  //Calcula ángulos con acelerómetro
  accel_ang_x=atan(ay/sqrt(pow(ax,2) + pow(az,2)))*(180.0/3.14);
  accel_ang_y=atan(-ax/sqrt(pow(ay,2) + pow(az,2)))*(180.0/3.14);
}

//---Función que obtiene el voltaje de la batería
float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  //Lectura (0-1023)-lectura mínima posible
  //Salida máxima (25v)-salida mínima (0v)
  //Entrada máxima (1023)-entrada mínima (0)
  //Salida mínima (0v) 
  return ((x-in_min)*(out_max-out_min)/(in_max-in_min)+out_min);
}

//---Función que obtiene el porcentaje de batería
float porcentaje(float x, float min, float max, float pmin, float pmax) {
  if (x>=max) { //Si el nivel de batería es mayor que el máximo volotaje definido
    return(pmax); //Regresa 100%
  } else if (x<=min) {  //Si el nivel de batería es menor que el mínimo voltaje definido
    return(pmin); //Regresa 0%
  } else {
    return ((x-min)*(pmax-pmin)/(max-min)+pmin);  //En otro caso, regresa el porcentaje
  }
}

//---Función que calcula el nivel de carga de la batería en porcentaje
float nivelBateria(int AX) {
  float prom=0;
  for (int i=0;i<n_muestras;i++) {
    vBat=fmap(analogRead(AX), 0, 1023, 0.00, 25.00)-0.19;  //Sensor lee de 0 a 25v
    prom=prom+vBat;
  }
  prom=prom/n_muestras;  
  return(porcentaje(prom, 12.00, 12.95, 0, 100));  //Guarda valor en nBat
}

//Funcion que reproduce la nota que se indique
void tono(uint16_t freq, uint16_t durac){
  unsigned long inicio_tiempo=millis(); //Guarda el momento en el que se llamó a la función
  unsigned long mitad=1000000L/freq/2;  //1MHz/frecuencia de la nota/2
  pinMode(buzz, OUTPUT); //Pone el pin como salida
  while (millis()-inicio_tiempo<durac) {  //Ej: 0-0<2000. Si eso se cumple ejecuta el programa
    digitalWrite(buzz, HIGH);
    delayMicroseconds(mitad);
    digitalWrite(buzz, LOW);
    delayMicroseconds(mitad);
    //Repoduce la nota
  }
  pinMode(buzz, INPUT);  //Cambia el pin a entrada
}

//Funcion que dice cual nota reproducir dependiendo del valor de b0
void bocina() {
  switch (b0) {
    case 0:
      //No llama a ninguna función
      break;
    case 1:
      //Sonido de correcto
      tono(NOTA_G5,150);
      tono(NOTA_C6,180);
      b0=0;
      break;
    case 2:
      //Sonido de error
      tono(NOTA_C4,700); //Reproduce la nota por los ms que se indiquen
      b0=0;
      break;
    case 3:
      //Tonos sistema de control (equilibrio)
      silencio=300;
      if (millis()>tiempo_tono+silencio){
        tiempo_tono=millis();
        tono(NOTA_C6,100); //Reproduce la nota por los ms que se indiquen
      }
      break;
    case 4:
      //Tonos sistema de control (equilibrio)
      silencio=1000;
      if (millis()>tiempo_tono+silencio){
        tiempo_tono=millis();
        tono(NOTA_E5,350); //Reproduce la nota por los ms que se indiquen
      }
      break;
  }
}

//---Función que realiza el cambio entre todos los posibles estados
void edos() {
  //Las entradas y salidas de la máquina de estados serían:
  //pulso_btn_up pulso_btn_dw s1 s21 s22 s23 s31 s32 s33 s4 s51 s52 s61 s62 / a1 a2
  switch (edo_actual) {
    case e0:
      //El cambio de estados de ciclo de subida, bajada o reset se encuentra en las interrupciones de los botones
      break;
  //------Estados de la secuencia de subida------//
    case up_e0:
      //Cambios en las variables
      pulso_btn_up=1; //Cambia variable para indicar que se pulsó btn de subir
      //¿Sensor 2 detecta que la altura es mayor a 15cm?
      //(Si la distancia medida por el sensor 2 está entre 10 y 15cm quiere decir que la silla está en posición para subir)
      distancia_u=leeUltra(2);
      op=0; //Imprime 1 vez
      if (distancia_u>sens_banq_s2-agregado_banq && distancia_u<sens_banq_s2+agregado_banq) {
        seq=edo_actual; //Para imprimir nota de secuencia
        op=0; //Imprime 1 vez
        edo_actual=up_e1; //Hace el cambio a e1
      } else {
        b0=2; //Sonido error
        seq=18; //Error/fuera de rango 1 posición array 18: Altura menor a 15cm
        op=0; //Imprime 1 vez
        edo_actual=rst_act;  //Reinicia
      }
      break;
    case up_e1:
      //Cambios en las variables
      s21=1;  //Variable >15cm
      s22=1;  //Variable de sila en posicion 
      distancia_u=leeUltra(1);
      op=0; //Imprime 1 vez
      //¿Sensor 1 detecta que la altura es menor a 30cm?
      //(Si la distancia medida por el sensor 1 es mayor a la distancia definida entre sensor y banqueta+agregado quiere decir que la banqueta mide <=30cm)
      if (distancia_u>sens_banq && distancia_u<tope) {
        b0=1; //Sonido correcto
        seq=edo_actual;
        op=0; //Imprime 1 vez
        edo_actual=up_e2; //Hace el cambio a e2
      } else {
        b0=2; //Sonido error
        seq=19; //Error/fuera de rango 2 posición array: 19: Altura mayor a 30cm
        op=0; //Imprime 1 vez
        edo_actual=rst_act;  //Reinicia
      }
      break;
    case up_e2:
      //Cambios en las variables
      s1=1;
      a1=1; a2=1; //Activa actuadores
      distancia_u=leeUltra(2);
      op=1; //Imprime cada 250ms
      ds2=distancia_u;
      //¿Sensor 2 detecta que subio la banqueta?
      //(Si sensor 2 detecta >45cm, quiere decir que ya subio la banqueta)
      if (ds2>45) {
        seq=edo_actual;
        op=0; //Imprime 1 vez
        edo_actual=up_e3; //Hace el cambio a e3
      }    
      break;
    case up_e3:
      //Cambios en las variables
      //Detiene cuando el sensor 2 ya no detecta banqueta
      s23=1;
      a1=0; a2=0; //Detiene los actuadores para guardar valor de distancia medida por sensor 3
      distancia_u=leeUltra(3);
      distancia_altura=distancia_u; //Guarda la distancia medida en la variable
      dist_banqueta=distancia_altura+dif_s2_s3;
      op=1; //Imprime cada 250ms
      //¿Distancia medida por el sensor 3 es menor que la distancia elevada + diferencia vertical entre sensor 2 y 3?
      if (distancia_u<dist_banqueta) {
        tam_banq=1; //Permite impresión de tamaño de banqueta
        seq=edo_actual;
        op=0; //Imprime 1 vez
        edo_actual=up_e4; //Hace el cambio a e4
      } else {
        //Si, por alguna razon, la silla ya esta en la altura correcta, pasa al e5 para evitar error
        seq=20; //Error/excepción 3 posición array 20: s3 indica elevación correcta, salto de estado
        op=0; //Imprime 1 vez
        edo_actual=up_e5; //Hace el cambio a e5 
      }
      break;
    case up_e4:
      //Cambios en las variables
      //Sube diferencia entre el s2 y s3
      tam_banq=0; //Cancela impresión del tamaño de banqueta 
      s31=1;
      a1=1; a2=1; //Activa los actuadores
      distancia_u=leeUltra(3);
      op=1; //Imprime cada 250ms
      //¿Distancia medida es mayor que la distancia que se elevo el modulo + diferencia vertical entre s2 y s3 + agregado?
      if (distancia_u>distancia_altura+dif_s2_s3+agregado_suelo) {
        seq=edo_actual;
        op=0; //Imprime 1 vez
        edo_actual=up_e5; //Hace el cambio al e5
      }
      break;
    case up_e5: 
      //Cambios en las variables
      //Detecta si el s3 ya está midiedo la banqueta 
      s32=1;
      a1=0; a2=0; //Detiene los actuadores
      distancia_u=leeUltra(3);
      op=1; //Imprime cada 250ms
      ds3=distancia_u;
      //¿Sensor 3 detecta el rango entre distancia del sensor 3 al suelo +- agregado? 
      //Quiere decir que ya se subio a la banqueta
      if (ds3<mod_suelo_s3+2 && ds3>mod_suelo_s3-5) {
        seq=edo_actual;
        op=0; //Imprime 1 vez
        edo_actual=up_e6; //Hace el cambio al e6
      }
      break;
    case up_e6: 
      //Cambios en las variables
      //Detecta si las ruedas traseras ya están contraidas por completo
      s33=1;
      a1=2; //Contrae el actuador 1
      //¿Sensor 5 detecta que la altura es mayor a la distancia que se elevo el modulo + diferencia entre s2 y s3 + agregado?
      edo_actual=up_e61;
      op=0;
      break;
    case up_e61:
      delay(200);
      calcDatos();
      distancia_u=leeUltra(5);
      op=1;
      if (cA1<rangoAct) {  //***Cambiar aquí*** /*distancia_u>distancia_altura+dif_s2_s3-sens_suelo*/
        //Cambia de estado cuando detecta que el actuador terminó de contraerse
        seq=7;
        op=0; //Imprime 1 vez
        edo_actual=up_e7; //Hace el cambio a e7
      }
      break;
    case up_e7:
      //Cambios en las variables
      //Detecta cuando la mitad frontal del módulo ya pasó a la banqueta
      s51=1;
      a1=0; //Detiene el actuador 1
      distancia_u=leeUltra(4);
      op=1; //Imprime cada 250ms
      ds4=distancia_u;
      //¿Sensor 4 detecta que distancia está entre el rango de la distancia del s4 al suelo +- agregado? 8.5
      if (ds4<mod_suelo_s4+3.5 && ds4>mod_suelo_s4-5) {
        seq=edo_actual;
        op=0; //Imprime 1 vez
        edo_actual=up_e8; //Hace el cambio a e8
      }
      break;
    case up_e8: //***Utilizar sensores de corriente para la comparación en lugar del S6
      //Cambios en las variables
      s4=1;
      a2=2; //Contrae el actuador 2
      op=1; //Imprime cada 250ms
      //¿Sensor 6 detecta que la altura es mayor a la distancia que se elevo el modulo + diferencia entre s2 y s3 + agregado?
      edo_actual=up_e81;
      break;
    case up_e81:
      delay(200);
      calcDatos();
      distancia_u=leeUltra(6);
      op=1;
      if (cA2<rangoAct) { //***Cambiar aquí***  distancia_u>distancia_altura+dif_s2_s3-sens_suelo_s6
        //Cambia de estado cuando detecta que el actuador terminó de contraerse
        seq=9;
        op=0; //Imprime 1 vez
        edo_actual=up_e9; //Hace el cambio a e9
      }    
      break;
    case up_e9:
      //Cambios en las variables
      b0=1; //Sonido correcto
      s61=1;
      seq=edo_actual;
      op=0; //Imprime 1 vez
      edo_actual=rst_act;  //Reinicia
      break;
  //------Estados de la secuencia de bajada------//
    case dw_e0:
      //Cambios en las variables
      if (pulso_btn_dw==1) {
        tiempo_bajada=millis();
        pulso_btn_dw=2;
      }
      if (pulso_btn_dw==2 && millis()<tiempo_bajada+rango_espera) { //Espera diferencia de distancia por 5 segundos
        b0=4; //Sonido de tiempo de espera bajada
        //¿Sensor 6 detecta entre 15 y 30cm?
        distancia_d=leeUltra(6);
        op=0; //Imprime 1 vez
        if (distancia_d>=14+sens_suelo_s6 && distancia_d<=31+sens_suelo_s6) { 
            dist_banqueta=distancia_d;
            tam_banq=1; //Permite impresión de tamaño de banqueta
            seq=edo_actual;
            op=0; //Imprime 1 vez
            b0=1; //Sonido correcto
            edo_actual=dw_e1; //Hace el cambio a e1
            pulso_btn_dw=1;
          }
      } else if (pulso_btn_dw==2 && millis()>tiempo_bajada+rango_espera) { //Si pasan mas de 5 segundos marca error y reinicia
        b0=2; //Sonido error
        seq=21; //Error/fuera de rango 4 posición array 21: fuera del rango 15 a 30cm
        op=0; //Imprime 1 vez
        edo_actual=rst_act;  //Reinicia
      }
      break;
    case dw_e1:
      //Cambios en las variables
      tam_banq=0; //Cancela impresión de tamaño de banqueta
      s61=1;
      a2=1; //Acciona actuador 2
      tiempo_bajada=millis();
      edo_actual=dw_e12; //Hace el cambio a e12
      break;
    case dw_e12:
      distancia_d=leeUltra(6);
      op=1; //Imprime cada 250ms
      //¿Sensor 6 detecta valor menor a distancia del sensor al suelo + agregado? 
      if (distancia_d<sens_suelo_s6+6) {
        seq=12;
        op=0; //Imprime 1 vez
        edo_actual=dw_e2; //Hace el cambio a e2
      }
      if (millis()>tiempo_bajada+22000) {
        calcDatos();
        if (cA2>-rangoAct) {
          seq=12;
          op=0; //Imprime 1 vez
          edo_actual=dw_e2; //Hace el cambio a e2
        }
      }
      break;
    case dw_e2:
      //Cambios en las variables
      s62=1;
      a2=0; //Detiene actuador 2
      distancia_d=leeUltra(5);
      op=1; //Imprime cada 250ms
      //¿Sensor 5 detecta valor mayor a distancia del sensor al suelo + agregado? 
      if (distancia_d>sens_suelo+agregado) {
        seq=edo_actual;
        op=0; //Imprime 1 vez
        edo_actual=dw_e3; //Hace el cambio a e3
      }
      break;
    case dw_e3:
      //Cambios en las variables
      s51=1;
      a2=2; //Contrae actuador 2
      op=1; //Imprime cada 250ms
      //¿Sensor 4 detecta valor cercano al original?
      edo_actual=dw_e31;
      break;
    case dw_e31:
      delay(200);
      distancia_d=leeUltra(4);
      op=1;
      calcDatos();
      if (cA2<rangoAct) { //distancia_d<sens_suelo+agregado
        //Cambia de estado cuando detecta que el actuador terminó de contraerse
        seq=14;
        op=0; //Imprime 1 vez
        edo_actual=dw_e4; //Hace el cambio a e4
      }
      break;
    case dw_e4:
      //Cambio en las variables
      b0=1; //Sonido correcto
      s4=1;
      a2=0; //Detiene actuador 2
      seq=edo_actual;
      op=0; //Imprime 1 vez
      edo_actual=rst_act;
      break;
  //------Estados de la secuencia de reset------//
    case rst_act:
      //rst de emergencia
      a1=2; a2=2; //Contrae los actuadores
      seq=edo_actual;
      op=1; //Imprime 1 vez
      edo_actual=rst_act2;
      break;
    case rst_act2:
      delay(200);
      calcDatos();
      op=1;
      if ((cA1<rangoAct) && (cA2<rangoAct)) {  
        //Cambia de estado cuando detecta que los dos actuadores terminaron de contraerse
        seq=22;
        op=0; //Imprime 1 vez
        edo_actual=rst_fin;
      }
      break;
    case rst_fin:
      seq=edo_actual;
      op=0; //Imprime 1 vez
      //Sigue contrayendo los actuadores por un tiempo
      delay(contraer_rst);
      //Regresa las variables a cero y a e0 
      variablesCero();
      break;
    default:
      //Error
      break;
  } 
}

//---Función que detecta cambios de estado y regresa valores a 0 para no imprimir datos que no son se usarán en estadistica
void edoStrings() {
  //Si hay un cambio de estados imprime en Serial
  if (aux_cont!=edo_actual) {
    if (edo_actual==e0 && aux_cont!=e0) {
      impr_encabezado=1;  //Indica que se imprima el encabezado 
    }
    e_actual=edo_actual;  //Variable para escoger lo que se va a imprimir en el array del estado actual
  } 
  sens=edo_actual;  //Para elegir el string a imprimir de las lecturas de sensores
  seq=0;  //Regresa a 0 para no imprimir notas en cada loop
  aux_cont=edo_actual;  //Guarda el estado nuevo
}

//---Función que calcula
void calcDatos(){
  cA1=getCorriente(c1);  //Obtiene corriente del actuador 1
  cA2=getCorriente(c2);  //Obtiene corriente del actuador 2
  nBat=nivelBateria(vb);  //Obtiene nivel de bateria
  angulos();  //Obtiene ángulos (X, Y)
  //Serial.println("\nA1: "+String(cA1)+"\tA2: "+String(cA2)+"\tBat: "+String(nBat));
}

//---Función que toma la decisión de qué imprimir y cada cuando hacerlo
void imprDatos(int x){
  //Se utilizan punteros para evitar errores de impresión y evitar el uso de más memoria al crear variables que guarden los datos 
  if (pulso_btn_up==1) {
    ptr_datos[2]=&distancia_u; //ptr[0] apunta a distancia_u para imprimirlo después
  } else if(pulso_btn_dw==1) {
    ptr_datos[2]=&distancia_d;  //ptr[0] apunta a distancia_d para imprimirlo después
  } 
  ptr_datos[1]=&dist_banqueta;  //ptr[1] apunta a distancia_altura para imprimirlo después 
  calcDatos(); //Obtiene las corrientes, nivel de batería y ángulos
  ptr_datos[0]=&cont_pruebas; //ptr_datos[0] apunta a cont_pruebas
  ptr_datos[3]=&cA1;
  ptr_datos[4]=&cA2;
  ptr_datos[5]=&accel_ang_x;
  ptr_datos[6]=&accel_ang_y;
  ptr_datos[7]=&nBat;
  //Imprime por Serial
  if(impr_encabezado==1) {
    for(int i=0;i<10;i++) {
      if(opcion==1) {
        Serial.print(encabezado_array[i]+"\t"); //Imprime encabezado por Serial
      } else if (opcion==2) {
        BT.print(encabezado_array[i]+"\t"); //Imprime encabezado por BT
      } else {
        //No imprime nada
      }
    }
    impr_encabezado=0;  //Regresa a 0 para no volverlo a imprimir
    cont_pruebas=cont_pruebas+1;  //Variable para ir contando las pruebas que se hagan cada que el estado regrese a e0
    impSerialBT();  //Imprime una linea de datos
  }

  //Si x=0 no hace impresión cada 250ms, si es 1 impresión cada 250ms, si es otro no imprime
  if (x==0){
    //Util para estados donde se está menos de 250ms
    impSerialBT();  //Imprime linea de datos
    op=2;
  } else if (x==1){
    if (millis()>tiempo_impr+periodo_impr) { //Si es mayor que 250ms
      tiempo_impr=millis(); //Guarda tiempo actual
      impSerialBT(); //Imprime linea de datos
    }
  } else {
    //No hace nada
  }
}


//---Función que imprime por Serial/BT
void impSerialBT () {
  //"\n No. Prueba \t Tamaño banqueta (cm) \t Estado actual \t Lectura sensor (cm) \t Corriente act1 (A) \t Corriente act2 (A) \t Inclinación (X°) \t Inclinación (Y°) \t Batería (%) \t Nota de secuencia"
  if(opcion==1) {
    //Imprime por Serial
    Serial.print("\n\t"
    +String(*ptr_datos[0],0)+"\t"  //No. prueba sin decimales
    +String(*ptr_datos[1],2)+"\t"  //Tamaño de banqueta
    +String(e_actual)+e_actual_array[e_actual]+"\t"  //Estado actual
    +sens_array[sens]+String(*ptr_datos[2])+"\t" //Medición del sensor en el estado actual
    +String(*ptr_datos[3],2)+"\t"  //Corriente del actuador 1
    +String(*ptr_datos[4],2)+"\t"  //Corriente del actuador 2
    +String(*ptr_datos[5],1)+"°\t"+String(*ptr_datos[6],1)+"°\t"  //Inclinación X° \t Inclinación Y° \t
    +String(*ptr_datos[7],0)+"\t" //Bateria
    +seq_array[seq]+"("+String(pulso_btn_up)+String(pulso_btn_dw)+String(n_pulsos,0)+")"
    );
  } else if (opcion==2) {
    //Imprime por BT
    BT.print("\n\t"
    +String(*ptr_datos[0],0)+"\t"  //No. prueba sin decimales
    +String(*ptr_datos[1],2)+"cm\t"  //Tamaño de banqueta
    +String(e_actual)+e_actual_array[e_actual]+"\t"  //Estado actual
    +sens_array[sens]+String(*ptr_datos[2])+"cm\t" //Medición del sensor en el estado actual
    +String(*ptr_datos[3],2)+"\t"  //Corriente del actuador 1
    +String(*ptr_datos[4],2)+"\t"  //Corriente del actuador 2
    +String(*ptr_datos[5],1)+"°\t"+String(*ptr_datos[6],1)+"°\t"  //Inclinación X° \t Inclinación Y° \t
    +String(*ptr_datos[7],0)+"\t" //Bateria
    +seq_array[seq]+"("+String(pulso_btn_up,0)+","+String(pulso_btn_dw,0)+","+String(n_pulsos,0)+")"
    );
  } else {
    //No imprime nada
  }
  
}
void sistemaControl(){
  angulos();
  //el sistema de control se activa cuando no se está en reposo  
  if (edo_actual!=e0) {
    //contrrol para cilo de subida 
    //comparacion util sólo en el ciclo de subida 
    if(edo_actual==up_e6 || edo_actual==up_e61 || edo_actual==up_e7){
      if (accel_ang_x<subidaideal) {
        b0=3; //Sonido sistema de control
        a2=1;
      } else{
        b0=0; //Desactiva sonido
        a2=0;
      }
    }
    if(edo_actual==dw_e3 || edo_actual==dw_e31){
      if(accel_ang_x>bajadaideal){
        b0=3; //Sonido sistema de control
        a1=1;
        a2=4;
      }else{
        b0=0; //Desactiva sonido 
        a1=2;
        a2=2;
      }
    }
  }
}
//---Terminan funciones de operación---///
//------Terminan funciones------//

//--------------------------------------------------

void setup() {
  Serial.begin(9600); //Establece los baudios a 9600 para la comunicación serial
  BT.begin(9600); //Establece a 9600 para comunicación BT
  inicio(); //Ejecuta función para inicializar entradas y salidas
}

void loop() {
  edos(); //Mantiene en ciclo la llamada a la función de cambios de estados
  sistemaControl(); //Llama al sistema de control después de la elección de los estados para hacer los ajustes a las variables de los actuadores
  imprDatos(op);  //Función que imprime 1 vez o cada 250ms dependiendo de la variable op que cambia en cada estado
  edoStrings(); //Detección de cambios de estados para la impresión y ajuste de Strings 
  act1();    //Funcion para activar actuador 1
  act2();   //Función para activar actuador 2
  bocina(); //Funcion de bocina
}
