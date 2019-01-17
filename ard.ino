
//
// *******************************
// ***        ATTENTION        ***
// ***    la    1ere    fois   ***
// *** commencer par installer ***
// ***        eeprom.ino       ***
// *******************************
//
// Pilotage carte arduino
// Version: 0.1
// Protocol 0.1
//

//
// ATTENTION: les broches 0 1 7 8 12 13 sont reservees a l'appli
// INTERDIT de les reconfigurer
//

//
// Entree: Capteur de temperature OneWire (DS18B20) connecte sur D12
//
// Temperature Boitier
// Temperature Radiateur 1
// Temperature Radiateur 2
// Temperature Reservoir 1
// Temperature Reservoir 2
// Temperature Interieur (chambre)
// Temperature Exterieur
//

//
// Entree: Capteur analogique
//
// Lecture debit pompe connecte sur A6
// <-- Transforme info vitesse rotation (capteur debit pompe) en tension analogique
// Capteur luminosite connecte sur A7
// --> Pilotage intensite led RGB en fonction de la luminosite ambiante (photoresistance)
//

//
// Entree: Capteur numerique
//
// Interrupteur marche/arret connecte sur D2 (int)
//

//
// Sortie: PWM
//
// Luminosite	  led				RGB				connecte sur D3
// Vitesse		  Ventilateur		Boitier			connecte sur D5
// Vitesse		  Ventilateur		Radiateur 1		connecte sur D6
// Vitesse		  Ventilateur		Radiateur 2		connecte sur D9
// Alimentation	  Peltier 1   		Reservoir 1		connecte sur D10
// Alimentation	  Peltier 2   		Reservoir 2		connecte sur D11
//

//
// Sortie Numerique connecte sur A0 A1 A2 A3 (attention pas entree analogique)
//
// Led Bleue temperature Boitier <20°C				connecte sur A0
// Led Verte temperature Boitier >20°C et <30°C		connecte sur A1
// Led Rouge temperature Boitier >30°C				connecte sur A2
// Pilotage de la pompe 1			    			connecte sur A3
//

//
// Vrai sortie numerique
//
// voyant erreur debit pompe  						connecte sur D4
// Si capteur rotatif de debit est arrete -> erreur

//
// Sortie Numerique connecte sur D13 (LED carte arduino)
//
// Montre le bon fonctionnement de la carte
// et pilote un watchdog electronique
//

//
// Liaison I2C
//
// SDA				connecte sur A4
// SCL				connecte sur A5

//
// Port serie connecte sur D0 D1 + handshaking D7 D8
//
// Communication avec ordi
//
// send:
// ~~~~~
// AB=C
// A: capteur/commande
// B: numero du capteur/commande
// C: valeur lue du capteur ou envoyee a la sortie commandee (en hexa sur 2c.)
// A en majus lecture capteur
// T.=.. lecture hh sur capteur temperature n
// A.=.. lecture hh sur entree analogique n
// N.=.. lecture hh sur entree numerique n
// A en minus pilote une sortie (commande)
// a.=.. pilote hh sur sortie analogique n (pwm)
// n.=.. pilote hh sur sortie numerique n
// receive:
// ~~~~~~~~
// - ordi envoi de nouveau parametre dans la table SRT
// AB.C=D\n
// A: S
// B: numero de sortie de la table srt[.][]
// C: numero parametre de la table srt[][.]
// D: valeur en hexa a mettre dans la table srt[][]=..
// - ordi envoi une nouvelle adresse de capteur de temperature
// AB=C\n
// A: D
// B: numero capteur temperature (0-9)
// C: adresse du nouveau capteur (16c. hexa)
// - ordi veut connaitre un parametre de la table SRT
// envoi r:.,.\n
// reponse s=hh\n
// - ordi veut recevoir toute la table SRT
// envoi R\n
// reponse S\nhh\nhh\nhh\nhh\n...E\n
// - ordi veut connaitre une  adresse de la table ADR
// envoi c:.\n
// reponse d\nhh\nhh\nhh\nhh\nhh\nhh\nhh\nhh\nf\n
// - ordi veut recevoir toute la table ADR
// envoi C\n
// reponse D\nhh\nhh\nhh\nhh\nhh\nhh\nhh\nhh\n...F\n
// - ordi veut connaitre le nombre de capteur de temperature (reel)
// envoi P\n
// reponse Q=./. nombre capteur reellement trouve / configure dans la table
// - ordi veut connaitre le mode de fonctionnement des ventilo (silencieux/securise)
// envoi M\n
// reponse N=. 0=silencieux 1=securise
//

//
// LCD pour parametrage
// Clavier pour saisie parametre
//
// Reserve
//

//
// WatchDog reset si plante
//
// Version electronique car on utilise deja les 3 timers
// lecture sortie D13
//

//
// Simulateur d'entree connecte sur A3 (ATTENTION passage sortie numerique a entree analogique)
//
// un potentiometre permet de tester chaque fonction
// en simulant une valeur d'entree
// en modifiant la table de correspondance (send ordi)
//

#include <OneWire.h>
#include <EEPROM.h>


#define SECURITE 0																// mode silencieux (fonctionnement des ventilo et eclairage)
#define SILENCE  !SECURITE

#define NEGATIF 0																// mode suiveur (1) ou inverseur (0)
#define POSITIF 1

#define OFF 0																	// on off DTR_nano DSR_rs232
#define ON !OFF

#define ITP_NO  0																// digitale / analogique
#define ITP_DGT 1
#define ITP_ANL 2
#define ITP_OWR 4
#define OTP_NO  0
#define OTP_OFF 1
#define OTP_ON  2
#define OTP_DGT 4
#define OTP_PWM 8

#define LNG 19																	// Longueur message recu
#define NMB 10																	// Nombre max de capteur onewire
#define PRT_ACK 2																	// ack
#define PRT_NACK 3																	// nack

// definition de toutes les broches (nano)

#define PD0 0																	// pin D0  receive
#define PD1 1																	// pin D1  send
#define PD2 2																	// pin D2  Entree numerique D2 (int)
#define PD3 3																	// pin D3  Sortie analogique luminosite eclairage (PWM)
#define PD4 4																	// pin D4  Sortie numerique erreur debit pompe
#define PD5 5																	// pin D5  Sortie analogique ventilo boitier (PWM)
#define PD6 6																	// pin D6  Sortie analogique ventilo radiateur 1 (PWM)
#define PD7 7																	// pin D7  Entree numerique D7
#define PD8 8																	// pin D8  Sortie numerique D8
#define PD9 9																	// pin D9  Sortie analogique ventilo radiateur 2 (PWM)
#define PD10 10																	// pin D10 Sortie analogique peltier reservoir 1 (PWM)
#define PD11 11																	// pin D11 Sortie analogique peltier reservoir 2 (PWM)
#define PD12 12																	// pin D12 Entree capteur onewire
#define PD13 13																	// pin D13 Sortie numerique voyant carte + watchdog electronique
#define PD14 14																	// pin D14 | pin A0 led bleue
#define PD15 15																	// pin D15 | pin A1 led verte
#define PD16 16																	// pin D16 | pin A2 led rouge
#define PD17 17																	// pin D17 | pin A3 pilotage pompe
#define PD18 18																	// pin D18 | pin A4 reserve I2C
#define PD19 19																	// pin D19 | pin A5 reserve I2C
#define PA0 0																	// pin A0  | pin D14 Sortie numerique led bleue
#define PA1 1																	// pin A1  | pin D15 Sortie numerique led verte
#define PA2 2																	// pin A2  | pin D16 Sortie numerique led rouge
#define PA3 3																	// pin A3  | pin D17 Entree analogique simulateur valeur
#define PA4 4																	// pin A4  | pin D18 Entree analogique A4 ou I2C
#define PA5 5																	// pin A5  | pin D19 Entree analogique A5 ou I2C
#define PA6 6																	// pin A6  Entree analogique debit pompe
#define PA7 7																	// pin A7  Entree analogique luminosite (photoresistance)

// definition des broches utilisees

#define B_RXD 0																	// pin D0  rcv	receive
#define B_TXD 1																	// pin D1  snd	send
#define B_INT 2																	// pin D2  in	Bouton Marche/Arret D2 (int)
#define B_RGB 3																	// pin D3  pwm	luminosite led rgb
#define B_ERR 4																	// pin D4  out	Erreur pompe led erreur
#define B_VBT 5																	// pin D5  pwm	ventilo boitier
#define B_VR1 6																	// pin D6  pwm	ventilo radiateur 1
#define B_DSR 7																	// pin D7  in	DSR machine hote prete ?
#define B_DTR 8																	// pin D8  out	DTR carte prete
#define B_VR2 9																	// pin D9  pwm	ventilo radiateur 2
#define B_VP1 10																// pin D10 pwm	ventilo peltier 1
#define B_VP2 11																// pin D11 pwm	ventilo peltier 2
#define B_OWR 12																// pin D12 owr	Capteur temperature
#define B_WDG 13																// pin D13 out	Watchdog electronique (+ voyant carte)
#define B_LDB 14																// pin D14 out	led bleue temperature boitier (A0)
#define B_LDV 15																// pin D15 out	led verte temperature boitier (A1)
#define B_LDR 16																// pin D16 out	led rouge temperature boitier (A2)
#define B_PMP 17																// pin D17 out	alimentation pompe (A3)
#define B_SDA 18																// pin D18 i2c	reserve I2C (A4)
#define B_SCL 19																// pin D19 i2c	reserve I2C (A5)


// description contenu de la table (srt[.][]) numero de la sortie
#define S00 0																	// pwm luminosite piece
#define S01 1																	// pwm ventilo boitier
#define S02 2																	// pwm ventilo radiateur 1
#define S03 3																	// pwm ventilo radiateur 2
#define S04 4																	// pwm peltier reservoir 1
#define S05 5																	// pwm peltier reservoir 1
#define S06 6																	// dgt led bleue
#define S07 7																	// dgt led verte
#define S08 8																	// dgt led rouge
#define S09 9																	// dgt erreur debit pompe
#define S10 10																	// dgt alimentation pompe
#define S11 11																	// no
#define S12 12																	// no
#define S13 13																	// no
//#define S15 15																// no
//#define S16 16																// no
//#define S17 17																// no
//#define S18 18																// no
//#define S19 19																// no
//#define S20 20																// no
//#define S21 21																// no
//#define S22 22																// no
#define SRT 14																	// *** dernier pour creation table ***

// description contenu de la table (srt[][.]) mesure/parametre
#define P_ITP 0																	// type d'entree: numerique/analogique/temperature (in=no=0 in=dgt=1 in=anl=2 in=owr=4)
#define P_IBR 1																	// le capteur d'entree est sur la broche (0-99)
#define P_IMN 2																	// seuil de mesure mini (entree 0-1023)
#define P_IMX 3																	// seuil de mesure maxi (entree 0-1023)
#define P_OTP 4																	// type de sortie numerique/analogique (out=no=0 out=off=1 out=on=2 out=dgt=4 out=pwm=8)
#define P_OBR 5																	// la sortie est sur la broche (0-99)
#define P_OMN 6																	// seuil commande mini (sortie 0-255)
#define P_OMX 7																	// seuil commande maxi (sortie 0-255)
#define P_NEN 8																	// indice nom de l'entree
#define P_NCP 9																	// indice nom du capteur
#define P_NSR 10																// indice nom de la sortie
#define P_RSR 11																// reserve
#define P_MSR 12																// mesure lu par le capteur (0-1023)
#define P_CMM 13																// commande envoye sur la sortie (0-255)
#define P_PRM 14																// *** dernier pour creation table ***

union _gun																		// gestion du temps systeme
{
	unsigned long l;
	unsigned short s[2];
} gun;

unsigned short gs_n, gs_a;														// gestion du temps numerique et analogique
OneWire gs_onewire(B_OWR);														// capteur de temperature onewire (DS18B20) sur D12
unsigned char gc_o;																// numero du capteur de temperature (255 aucun 0 premier 1 deuxieme ...)
unsigned char gc_a;																// numero du capteur analogic courant (255 aucun 0 premier 1 deuxieme ...)
unsigned char gc_n;																// numero du capteur numeric courant (255 aucun 0 premier 1 deuxieme ...)
unsigned char gc_m;																// choix du mode de fonctionnement des ventilo 1=silence+nuit 0=securite+jour
uint8_t adr[NMB][8];															// adresse des capteurs de temperature
unsigned short srt[SRT][P_PRM];													// table valeur/parametre (voir #define)
char rcv[LNG+7] = "";															// buffer reception port serie (parametrage)
char wrk[LNG+1] = "";
unsigned char gr_prt[9] = {'\t', '\n', 6, 0x15, '}', '`', '~', 'x', 0};			// protocole
unsigned char gr_lng[21] = "abcdefghijklmnopqrst";								// longueur
unsigned char gr_ga[4] = "j B";													// groupe + numero emetteur + numero destinataire
unsigned char gc_x;																// indice rcv[]
unsigned char gc_f;																// phase de reception
unsigned char gc_r;																// commande recu

void fv_chradr(void);															// ajout capteur temperature
void fv_chrsrt(void);															// charge table srt[][] a partir de l'eeprom
void fv_cntbrc(void);															// controle l'unicite des sorties
unsigned char fc_nmrscr(void);													// lecture valeur capteur numerique
unsigned char fc_anlscr(void);													// lecture valeur capteur analogique
unsigned char fc_tmpscr(void);													// lecture valeur capteur de temperature
unsigned char fc_cmmscr(char pc_t, char pc_c);									// calcul de la commande en fonction de la valeur du capteur courant
void fv_send(char, char, char, unsigned char, unsigned char);					// envoi de message a l'hote

void setup(void) 
{
	Serial.begin(115200);
	//pinMode(B_RXD, INPUT);													// inutile car fait par Serial.begin()
	//pinMode(B_TXD, OUTPUT);													// inutile car fait par Serial.begin()
	//pinMode(B_INT, INPUT);													// inutile fait par le programme
	//pinMode(B_RGB, OUTPUT);													// inutile car fait par analogWrite()
	//pinMode(B_ERR, OUTPUT);													// inutile fait par le programme
	//pinMode(B_VBT, OUTPUT);													// inutile car fait par analogWrite()
	//pinMode(B_VR1, OUTPUT);													// inutile car fait par analogWrite()
	pinMode(B_DSR,	 INPUT);
	pinMode(B_DTR,   OUTPUT);
	//pinMode(B_VR2, OUTPUT);													// inutile car fait par analogWrite()
	//pinMode(B_VP1, OUTPUT);													// inutile car fait par analogWrite()
	//pinMode(B_VP2, OUTPUT);													// inutile car fait par analogWrite()
	//pinMode(B_OWR, INPUT);													// inutile car fait par onewire
	pinMode(B_WDG,	 OUTPUT);
	//pinMode(B_LDB, OUTPUT);													// inutile fait par le programme
	//pinMode(B_LDV, OUTPUT);													// inutile fait par le programme
	//pinMode(B_LDR, OUTPUT);													// inutile fait par le programme
	//pinMode(B_PMP, OUTPUT);													// inutile fait par le programme
	//pinMode(B_SDA, INPUT);													// inutile car fait par i2c()
	//pinMode(B_SCL, INPUT);													// inutile car fait par i2c()
	//pinMode(PA6,	 INPUT);													// inutile car fait par analogRead()
	//pinMode(PA7,	 INPUT);													// inutile car fait par analogRead()

	digitalWrite(B_DTR, ON);													// carte non prete DTR=1 dsr_rs232=rouge
	
	for (gc_x=0; gc_x<SRT; gc_x++)												// Initialisation srt[][P_MSR] et srt[][P_CMM]
		srt[gc_x][P_MSR] = srt[gc_x][P_CMM] = 0;
	
	fv_chradr();																// Chargement table adr a partir des nouveaux capteurs trouves
	
	// Configuration capteur en  9bits (temps lecture =  94mS)
	// Configuration capteur en 10bits (temps lecture = 188mS)
	// Configuration capteur en 11bits (temps lecture = 375mS)
	// Configuration capteur en 12bits (temps lecture = 750mS)
	
	// gc_o = 255 <==> il n'y a pas de capteur de temperature onewire
	// gc_o = ... <==> selectionne le premier capteur onewire de adr
	
	for (gc_x=0,gc_o=255; gc_x<NMB; gc_x++)
	{
		if (adr[gc_x][0] == 0x28)
		{
			if (gc_o == 255)
				gc_o = gc_x;
			gs_onewire.reset();
			gs_onewire.select(adr[gc_x]);
			gs_onewire.write(0x4E);
			gs_onewire.write(0x00);
			gs_onewire.write(0x00);
			gs_onewire.write(0x3F); // 0 R1 R0 1 1 1 1 1 (R1R0= 00=9 bits, 01=10 bits, 10=11 bits, 11=12 bits)
		
			delay(200);
			digitalWrite(B_WDG, !digitalRead(B_WDG));							// cligno voyant carte (D13)
		}
	}
	
	fv_chrsrt();																// Chargement table srt a partir de l'eeprom (info=0 offset=32)
	
	fv_cntbrc();																// Controle les broches de sorties (sortie unique et pas en entree)

	for (gc_x=0; gc_x<SRT; gc_x++)												// Configuration des broches
	{
		if (srt[gc_x][P_OTP] == OTP_DGT)										// broche utilisee en sortie
			pinMode(srt[gc_x][P_OBR], OUTPUT);
		if (srt[gc_x][P_ITP] == ITP_DGT)										// broche utilisee en entree
			pinMode(srt[gc_x][P_IBR], INPUT);
	}

	gc_m = SILENCE;																// Initialisation des variables globales
	gc_n = 255;
	gc_a = 255;
	gc_x = 0;
	gc_f = 0;
	gc_r = 0;
	gs_n = 0;
	gs_a = 0;
		
	digitalWrite(B_DTR, OFF);													// carte prete (handshaking) DTR=0 DSR_rs232=vert
}

void loop(void) 
{
	unsigned char lc_a, lc_b, vlr[2];
	unsigned short ls_c;
	byte lr_adr[8];

	if (gc_r != 0)																										// un message a ete recu
	{
		switch (wrk[1])																									// quoi ?
		{
			case 'x':																									// table srt
				switch (wrk[0])
				{
					case '{':																							// lecture
						fv_send(6, wrk[2]-'a', wrk[3]-'a', ' ' + (srt[wrk[2]-'a'][wrk[3]-'a'] >> 6), ' ' + (srt[wrk[2]-'a'][wrk[3]-'a'] & 0x3F));
						break;

					case '}':																							// modif
						ls_c = wrk[4] - ' ';
						ls_c <<= 6;
						ls_c += wrk[5] - ' ';
						ls_c &= 0x3FF;
						srt[wrk[2]-'a'][wrk[3]-'a'] = ls_c;
						fv_send(1, PRT_ACK, 0, 0, 0);
						break;
					
					case '|':																							// modif + eeprom
						ls_c = wrk[4] - ' ';
						ls_c <<= 6;
						ls_c += wrk[5] - ' ';
						ls_c &= 0x3FF;
						srt[wrk[2]-'a'][wrk[3]-'a'] = ls_c;
						if (((wrk[2]>='a') && (wrk[2]<='g')) && ((wrk[3]>='a') && (wrk[3]<='k')))
						{
							if (ls_c == 0x3FF)
								EEPROM.write((wrk[2]-'a')*SRT+(wrk[3]-'a')+88, 255);
							else if (ls_c < 128)
								EEPROM.write((wrk[2]-'a')*SRT+(wrk[3]-'a')+88, ls_c);
							else
								EEPROM.write((wrk[2]-'a')*SRT+(wrk[3]-'a')+88, ls_c / 4);
						}
						fv_send(1, PRT_ACK, 0, 0, 0);
						break;
				}
				break;
				
			case 'w':																									// table adr
				switch (wrk[0])
				{
					case '{':																						// lecture
						Serial.write(&gr_prt[0], 1);
						Serial.write(&gr_lng[19], 1);
						Serial.write(gr_ga, 3);
						Serial.write(&gr_prt[4], 1);
						Serial.write((const uint8_t *) &wrk[1], 2);
						for (lc_a=0; lc_a<8; lc_a++)
						{
							vlr[0] = ' ' + (adr[wrk[2]-'a'][lc_a] >> 6);
							vlr[1] = ' ' + (adr[wrk[2]-'a'][lc_a] & 0x3F);
							Serial.write(vlr, 2);
						}
						Serial.write(&gr_prt[1], 1);
						break;

					case '}':																						// modif
						for (lc_a=0; lc_a<8; lc_a++)
						{
							ls_c = wrk[lc_a*2+3] - ' ';
							ls_c <<= 6;
							ls_c += wrk[lc_a*2+4] - ' ';
							ls_c &= 0xFF;
							adr[wrk[2]-'a'][lc_a] = (int8_t) ls_c;
						}
						fv_send(1, PRT_ACK, 0, 0, 0);
						break;

					case '|':																							// modif + eeprom
						for (lc_a=0; lc_a<8; lc_a++)
						{
							ls_c = wrk[lc_a*2+3] - ' ';
							ls_c <<= 6;
							ls_c += wrk[lc_a*2+4] - ' ';
							ls_c &= 0xFF;
							adr[wrk[2]-'a'][lc_a] = (int8_t) ls_c;
						}
						fv_send(1, PRT_ACK, 0, 0, 0);
						break;
				}
				break;

			case 'y':																									// demande nombre capteur trouve_reel conf_dans_table maxi_possible_table
				switch (wrk[0])
				{
					case '{':																							// lecture
						Serial.write(&gr_prt[0], 1);
						Serial.write(&gr_lng[6], 1);
						Serial.write(gr_ga, 3);
						Serial.write(&gr_prt[4], 1);
						Serial.write((const uint8_t *) &wrk[1], 1);
						for (lc_a=lc_b=0; lc_a<SRT; lc_a++)
							if (srt[lc_a][P_ITP] == ITP_DGT)
								lc_b++;
						Serial.write(&gr_lng[lc_b], 1);
						for (lc_a=lc_b=0; lc_a<SRT; lc_a++)
							if (srt[lc_a][P_ITP] == ITP_ANL)
								lc_b++;
						Serial.write(&gr_lng[lc_b], 1);
						
						gs_onewire.reset_search();																		// premier
						lc_a = 0;
						while (gs_onewire.search(lr_adr))																// pour tous les capteurs
							if (gs_onewire.crc8(lr_adr, 7) == lr_adr[7])
								lc_a++;
								
						Serial.write(&gr_lng[lc_a], 1);
						for (lc_a=lc_b=0; lc_a<NMB; lc_a++)
							if (adr[lc_a][0] == 0x28)
								lc_b++;
						Serial.write(&gr_lng[lc_b], 1);
						Serial.write(&gr_prt[1], 1);
						break;

					default:
						fv_send(1, PRT_NACK, 0, 0, 0);
						break;
				}
				break;
				
			case 'z':																									// mode silencieux (ventilo+eclairage)
				switch (wrk[0])
				{
					case '{':																							// lecture
						Serial.write(&gr_prt[0], 1);
						Serial.write(&gr_lng[3], 1);
						Serial.write(gr_ga, 3);
						Serial.write(&gr_prt[4], 1);
						Serial.write((const uint8_t *) &wrk[1], 1);
						if (gc_m == SECURITE)
							Serial.write(&gr_prt[6], 1);
						else
							Serial.write(&gr_prt[5], 1);
						Serial.write(&gr_prt[1], 1);
						break;

					case '}':																							// modif
						if (wrk[2] == '~')
							gc_m = SECURITE;
						else
							gc_m = SILENCE;
						fv_send(1, PRT_ACK, 0, 0, 0);
						break;

					case '|':																							// modif + eeprom
						if (wrk[2] == '~')
							gc_m = SECURITE;
						else
							gc_m = SILENCE;
						fv_send(1, PRT_ACK, 0, 0, 0);
						break;
				}
				break;
		}
		
		gc_r = 0;
	}
	
	gun.l = millis();

	if ((gun.s[0] & 0xFF80) == (gs_n & 0xFF80))																			// moins de 128mS
		return;
	gs_n = gun.s[0];

	if ((lc_a = fc_nmrscr()) != 255)																					// lecture capteur numeric
		fc_cmmscr(ITP_DGT, lc_a);																						// ajustement des sorties liees au capteur numeric

	if ((gun.s[0] & 0xFF00) == (gs_a & 0xFF00))																			// moins de 256mS
		return;
	gs_a = gun.s[0];
	
	digitalWrite(B_WDG, !digitalRead(B_WDG));																			// cligno voyant carte (D13)

	if ((lc_a = fc_anlscr()) != 255)																					// lecture capteur analogic
		fc_cmmscr(ITP_ANL, lc_a);																						// ajustement des sorties liees au capteur analogic

	if (gc_o != 255)
		if ((lc_a = fc_tmpscr()) != 255)																				// lecture capteur temperature
			fc_cmmscr(ITP_OWR, lc_a);																					// ajustement des sorties liees au capteur de temperature
}

// Ajout nouveau capteur dans la table adr

void fv_chradr(void)
{
	unsigned char y, x, a, n;
	byte lr_adr[8];

	// ajoute les capteurs trouves qui ne sont pas encore en eeprom (capteur de test, remplacement, essai ...)
	
	gs_onewire.reset_search();																							// premier
	while (gs_onewire.search(lr_adr))																					// pour tous les capteurs
	{
		if (gs_onewire.crc8(lr_adr, 7) == lr_adr[7])
		{
			// si n'existe pas deja
			for (y=0; y<n; y++)
			{
				for (x=0; x<8; x++)
				{
					if (lr_adr[x] != adr[y][x])
						break;
				}
				if (x == 8)
					break;
			}
			if (y == n)
			{
				for (x=0; x<8; x++)																						// ajout
					adr[n][x] = lr_adr[x];
				n++;
			}
		}
	}
	
	digitalWrite(B_WDG, !digitalRead(B_WDG));																				// cligno voyant carte (D13)
}

// Chargement table srt a partir de l'eeprom
// les valeurs de l'eeprom sont en 8bits (0-255)
// les valeurs < 128 sont converties en 16 bits (0-127)
// les valeurs > 127 sont converties en 16 bits * 4 (512-1016)
// = 255 convertie en 0x3FF (1023)

void fv_chrsrt(void)
{
	unsigned char a, y, x, r;
	unsigned short v;
	
	for (y=0; y<SRT; y++)
	{
		for (x=r=0; x<P_RSR; r++,x++)
		{
			a = (y*16)+r+32;
			v = EEPROM.read(a);
			v &= 0xFF;
			srt[y][x] = v;
			if ((r==2) || (r==4) || (r==8) || (r==10))
			{
				r++;
				a = (y*16)+r+32;
				v = EEPROM.read(a);
				v &= 0xFF;
				srt[y][x] <<= 8;
				srt[y][x] |= v;
			}
		}
		if (srt[y][P_OTP] == 0)						// OTP == NO
		{
			srt[y][P_ITP] = 0;					// ITP
			srt[y][P_NEN] = 0;					// NEN
			srt[y][P_NCP] = 0;					// NCP
			srt[y][P_NSR] = 0;					// NSR
		}
	}
	
	digitalWrite(B_WDG, !digitalRead(B_WDG));																				// cligno voyant carte (D13)
}

//
// Controle broche entree / sortie
// Sortie doit etre unique (pas de double)
// Sortie ne doit pas etre configure en meme temps en entree
// Entree peut etre en double mais toujours du meme type (dgt anl owr ...)
// Attention exemple sortie D14 == entree A0
//

void fv_cntbrc(void)
{
	unsigned char lc_s, lc_y;
	
	for (lc_s=0; lc_s<SRT; lc_s++)																	// gestion sortie
	{
		for (lc_y=lc_s+1; lc_y<SRT; lc_y++)
		{
			if (lc_s == lc_y)
				continue;
				
			// a verifier que obr = 14 pour a0 et non pas 0
			if (srt[lc_s][P_OBR] == srt[lc_y][P_OBR])
				;//erreur sortie en double
			else if ((srt[lc_y][P_IBR] != ITP_OWR) && (srt[lc_y][P_IBR] == srt[lc_y][P_OBR]))
				;//erreur meme broche configuree en entre ET en sortie;
				
			//ok
		}
	}

	for (lc_s=0; lc_s<SRT; lc_s++)																	// gestion entree
	{
		for (lc_y=lc_s+1; lc_y<SRT; lc_y++)
		{
			if (lc_s == lc_y)
				continue;
				
			// a verifier que obr = 14 pour a0 et non pas 0
			if ((srt[lc_s][P_IBR] == srt[lc_y][P_IBR]) && ((srt[lc_s][P_ITP] != srt[lc_y][P_ITP])))
				;//erreur entree identique de type different
				
			//ok
		}
	}
}

//
// Lecture d'un capteur numerique en mode securise (reecrit toujours dans srt[][P_MSR])
//
// gc_n = 255 <==> il n'y a pas d'entree numerique selectionne
// gc_n = ... <==> l'entree numerique selectionne
//
// retour lc_t=255=pas_de_numeric lc_t=...=entree_numeric_mise_a_jour --> sert a pilote cmmscr()
//
// en sortie gc_n la prochaine entree ou la meme si unique ou 255 si disparue
//

unsigned char fc_nmrscr(void)
{
	unsigned char lc_n;
	unsigned char lc_j;
	unsigned char lc_t;
	
	if (gc_n == 255)																									// pas encore (ou plus) d'entree numerique selectionnee
	{
		for (lc_j=0; lc_j<SRT; lc_j++)																					// recherche si existe une entree numerique
			if (srt[lc_j][P_ITP] == ITP_DGT)
				break;
		if (lc_j == SRT)																								// non
			return(255);
			
		gc_n = srt[lc_j][P_IBR];																					// oui = selectionne
	}

	for (lc_j=0,lc_t=255; lc_j<SRT; lc_j++)																				// pour toutes les sorties
	{
		if (srt[lc_j][P_IBR] == gc_n)																				// sortie avec notre capteur (gc_n) en entree
		{
			if (lc_t == 255)
			{
				lc_n = digitalRead(gc_n);																				// lecture numerique (gc_n)
				lc_t = gc_n;
			}
			
			if (lc_n == 0)
			{
				srt[lc_j][P_MSR] = 0;																						// MAJ srt[P_MSR][] meme si identique
				fv_send(5, lc_j, P_MSR, gr_prt[6], 0);																	// envoi MSR a OFF
			}
			else
			{
				srt[lc_j][P_MSR] = 0x3FF;																					// MAJ srt[P_MSR][] meme si identique
				fv_send(5, lc_j, P_MSR, gr_prt[5], 0);																	// envoi MSR a ON
			}
		}
	}

	for (lc_j=0; lc_j<SRT; lc_j++)																						// recherche la permiere sortie qui utilise le capteur numerique courant
		if (srt[lc_j][P_IBR] == gc_n)
			break;
	for (lc_j++; lc_j<SRT; lc_j++)																						// recherche dans les sorties suivantes
		if ((srt[lc_j][P_ITP] == ITP_DGT) && (srt[lc_j][P_IBR] != gc_n))												// arret sur l'entree numerique differente suivante
			break;
	if (lc_j >= SRT)																									// l'entree numerique courante etait la derniere
		for (lc_j=0; lc_j<SRT; lc_j++)
			if (srt[lc_j][P_ITP] == ITP_DGT)																				// recherche de la premiere
				break;
	if (lc_j >= SRT)
		gc_n = 255;
	else
		gc_n = srt[lc_j][P_IBR];
	
	return(lc_t);
}

//
// Lecture d'un capteur analogique en mode securise (reecrit toujout dans srt[P_MSR][])
//
// gc_a = 255 <==> il n'y a pas d'entree analogique utilisee
// gc_a = ... <==> selectionne la premiere entree analogique
//
// retour: 255 si pas de capteur 0... numero capteur si trouve
//

unsigned char fc_anlscr(void)
{
	unsigned short ls_a;
	unsigned char  lc_j;
	unsigned char  lc_t;
	unsigned char  vlr[2];
	
	if (gc_a == 255)
	{
		for (lc_j=0; lc_j<SRT; lc_j++)
			if (srt[lc_j][P_ITP] == ITP_ANL)
				break;
		if (lc_j == SRT)
			return(255);

		gc_a = srt[lc_j][P_IBR];
	}

	for (lc_j=0,lc_t=255; lc_j<SRT; lc_j++)
	{
		if (srt[lc_j][P_IBR] == gc_a)
		{
			if (lc_t == 255)
			{
				ls_a = analogRead(gc_a);																				// lecture analogique (PA3 ... PA7) 10bits
				lc_t = gc_a;
			}
			
			srt[lc_j][P_MSR] = ls_a & 0x3FF;																				// convertion en 10bits
			fv_send(6, lc_j, P_MSR, ' ' + ((ls_a >> 6) & 0xF), ' ' + (ls_a & 0x3F));										// envoi en 10bits
		}
	}

	for (lc_j=0; lc_j<SRT; lc_j++)
		if (srt[lc_j][P_IBR] == gc_a)
			break;
	for (lc_j++; lc_j<SRT; lc_j++)
		if ((srt[lc_j][P_ITP] == ITP_ANL) && (srt[lc_j][P_IBR] != gc_a))
			break;
	if (lc_j >= SRT)
		for (lc_j=0; lc_j<SRT; lc_j++)
			if (srt[lc_j][P_ITP] == ITP_ANL)
				break;
	if (lc_j >= SRT)
		gc_a = 255;
	else
		gc_a = srt[lc_j][P_IBR];

	return(lc_t);
}

//
// Lecture d'un capteur de temperature
// retour: 255 si pas de capteur 0... numero capteur si trouve
//

unsigned char fc_tmpscr(void)
{	
	byte lr_data[9];
	byte lc_i;
	byte lc_t;
	int16_t lc_r;
	unsigned char lc_j, vlr[2];

	if (!gs_onewire.reset())																							// demande de lecture
		return(255);
	gs_onewire.select(adr[gc_o]);
	gs_onewire.write(0xBE);
	
	for (lc_i=0; lc_i<9; lc_i++)																						// lecture temperature
		lr_data[lc_i] = gs_onewire.read();
	if (gs_onewire.crc8(lr_data, 8) != lr_data[8])
		return(255);

	//switch (adr[gc_o][0]) 
	//{
	//	case 0x10:
			//Serial.println(" DS18S20");
	//		lc_t = 0;
	//		break;
	//	case 0x28:
			//Serial.println(" DS18B20");
	//		lc_t = 1;
	//		break;
	//	case 0x22:
			//Serial.println(" DS1822");
	//		lc_t = 1;
	//		break;
	//	default:
			//Serial.println(" Inconnu");
	//		lc_t = 255;
	//		break;
	//} 

    lc_r = (lr_data[1] << 8) | lr_data[0];
    
   	switch (lr_data[4] & 0x60)
   	{
   		case 0:																											//  9 bits 93.75 ms
			lc_r >>= 3;
   			break;
   		case 0x20:																										// 10 bits 187.5 ms
			lc_r >>= 2;
   			break;
   		case 0x40:																										// 11 bits 375 ms
			lc_r >>= 1;
   			break;
   		//case 0x60:																									// 12 bits 750 ms
   		//	break;
   	}
    lc_r &= 0x3FF;
    
    lc_t = 255;																											// valeur inutilise en sortie
    for (lc_i=0; lc_i<SRT; lc_i++)
    {
    	if (srt[lc_i][P_IBR] == gc_o)
    	{
			srt[lc_i][P_MSR] = lc_r;
			lc_t = gc_o;																								// ok nouvelle valeur dans MSR

			fv_send(6, lc_i, P_MSR, ' ' + ((lc_r >> 6) & 0xF), ' ' + (lc_r & 0x3F));
    	}
    }
	
	for (lc_j=gc_o+1; lc_j<NMB; lc_j++)
		if (adr[lc_j][0] == 0x28)
			break;
	if (lc_j == NMB)
		for (lc_j=0; lc_j<NMB; lc_j++)
			if (adr[lc_j][0] == 0x28)
				break;
	gc_o = lc_j;

	if (!gs_onewire.reset())																							// selection nouveau capteur
		return(255);
	gs_onewire.select(adr[gc_o]);
	gs_onewire.write(0x44);																								// demande nouvelle convertion
	
	return(lc_t);
}


//
// calcul de la commande en mode securise
//
// en parametre on a le numero du capteur que l'on vient de lire et qui va nous permettre de modifier la ou les sorties correspondantes
// 0-9 si capteur de temperature
// a-z si capteur numerique
// A-Z si capteur analogique
//

unsigned char fc_cmmscr(char pc_t, char pc_c)
{
	int16_t li;
	char lc_i, lc_s;

	for (lc_i=0; lc_i<SRT; lc_i++)																						// pour toutes les sorties
	{
		if ((srt[lc_i][P_ITP] == pc_t) && (srt[lc_i][P_IBR] == pc_c))													// si la sortie a notre capteur en entree 
		{
			if (srt[lc_i][P_OTP] == OTP_PWM)																			// la sortie est analogique (pwm)
			{
				if (pc_t == ITP_DGT)																					// entree dgt et sortie pwm
				{
				}
				else																									// entree (anl ou owr) et sortie anl
				{
					if ((srt[lc_i][P_IMN] < srt[lc_i][P_IMX]) && (srt[lc_i][P_OMN] < srt[lc_i][P_OMX]))
						lc_s = POSITIF;
					else
						lc_s = NEGATIF;

					if (srt[lc_i][P_MSR] < srt[lc_i][P_IMN])
					{
						if (gc_m == SILENCE)																				// en mode silence les ventilo sont arretes si temperature mesure trop faible
						{
							if (lc_s == POSITIF)
								srt[lc_i][P_CMM] = 0;
							else
								srt[lc_i][P_CMM] = srt[lc_i][P_OMN];
						}
						else																								// en mode securite les ventilo sont vitesse mini si temperature trop faible
						{
							if (lc_s == POSITIF)
								srt[lc_i][P_CMM] = srt[lc_i][P_OMN];
							else
								srt[lc_i][P_CMM] = 0x3FF;
						}

					}
					else
					{
						if (srt[lc_i][P_MSR] > srt[lc_i][P_IMX])
						{
							if (gc_m == SILENCE)																			// en mode silence les ventilo vitesse limite si temperature trop forte
							{
								if (lc_s == POSITIF)
									srt[lc_i][P_CMM] = srt[lc_i][P_OMX];
								else
									srt[lc_i][P_CMM] = 0;
							}
							else																							// en mode securite les ventilo vitesse maxi si temperature trop forte
							{
								if (lc_s == POSITIF)
									srt[lc_i][P_CMM] = 0x3FF;
								else
									srt[lc_i][P_CMM] = srt[lc_i][P_OMX];
							}
						}
						else
						{
							li  = srt[lc_i][P_MSR] - srt[lc_i][P_IMN];
							li *= srt[lc_i][P_OMX] - srt[lc_i][P_OMN];
							li /= srt[lc_i][P_IMX] - srt[lc_i][P_IMN];
							li += srt[lc_i][P_OMN];
							srt[lc_i][P_CMM] = li & 0x3FF;
						}
					}
				}

				analogWrite(srt[lc_i][P_OBR], srt[lc_i][P_CMM]);
				fv_send(6, lc_i, P_CMM, ' ' + (srt[lc_i][P_CMM] >> 6), ' ' + (srt[lc_i][P_CMM] & 0x3F));
			}
			else																										// la sortie est numerique
			{
				if (srt[lc_i][P_ITP] != ITP_DGT) //pc_c < 'a')																	// entree anl ou owr sortie dgt (hysteresis entre les valeurs on fait rien)
				{
					if (srt[lc_i][P_IMN] == srt[lc_i][P_IMX])																	// un seul seuil de mesure (mini == maxi)
					{
						if (srt[lc_i][P_OMN] == srt[lc_i][P_OMX])																// un seul seuil de sortie (mini == maxi)
						{																									// <=> 2 seuils de menures mais pour la valeur du seuil et non pour l'intervaleur
							if (srt[lc_i][P_MSR] == srt[lc_i][P_IMN])
								srt[lc_i][P_CMM] = srt[lc_i][P_OMN];
							else
								srt[lc_i][P_CMM] = !srt[lc_i][P_OMN];
						}
						else																								// deux seuil de sortie (mini et maxi)
						{
							if (srt[lc_i][P_MSR] < srt[lc_i][P_IMN])
								srt[lc_i][P_CMM] = srt[lc_i][P_OMN];
							if (srt[lc_i][P_MSR] > srt[lc_i][P_IMX])
								srt[lc_i][P_CMM] = srt[lc_i][P_OMX];
						}
					}
					else																									// deux seuil de mesure (mini et maxi)
					{
						if (srt[lc_i][P_OMN] == srt[lc_i][P_OMX])																// un seul seuil de sortie (mini == maxi)
						{
							if ((srt[lc_i][P_MSR] > srt[lc_i][P_IMN]) && (srt[lc_i][P_MSR] < srt[lc_i][P_IMX]))
								srt[lc_i][P_CMM] = srt[lc_i][P_OMN];
							if ((srt[lc_i][P_MSR] < srt[lc_i][P_IMN]) || (srt[lc_i][P_MSR] > srt[lc_i][P_IMX]))
								srt[lc_i][P_CMM] = !srt[lc_i][P_OMN];
						}
						else																								// deux seuil de sortie (mini et maxi)
						{
							if (srt[lc_i][P_ITP] != ITP_DGT)	//pc_c < 'a')												// entree anl ou owr sortie dgt (hysteresis entre les valeurs on fait rien)
							{
								if (srt[lc_i][P_MSR] < srt[lc_i][P_IMN])
									srt[lc_i][P_CMM] = srt[lc_i][P_OMN];
								if (srt[lc_i][P_MSR] > srt[lc_i][P_IMX])
									srt[lc_i][P_CMM] = srt[lc_i][P_OMX];
							}
							else																							// entree et sortie dgt
							{
								if (srt[lc_i][P_IMN] == srt[lc_i][P_OMN])														// ordre identique on.off et on.off OU off.on et off.on
								{
									if (srt[lc_i][P_MSR] == srt[lc_i][P_OMN])
										srt[lc_i][P_CMM] = srt[lc_i][P_OMN];
									else
										srt[lc_i][P_CMM] = srt[lc_i][P_OMX];
								}
								else																						// ordre inverse on.off et off.on OU off.on et on.off
								{
									if (srt[lc_i][P_MSR] == srt[lc_i][P_OMN])
										srt[lc_i][P_CMM] = srt[lc_i][P_OMX];
									else
										srt[lc_i][P_CMM] = srt[lc_i][P_OMN];
								}
							}
						}
					}
				}
				else																									// entree et sortie dgt
				{
					if (srt[lc_i][P_IMN] == srt[lc_i][P_IMX])																// seuil mesure on=on OU off=off
					{
						if (srt[lc_i][P_OMN] == srt[lc_i][P_OMX])																// un seul seuil de sortie (mini == maxi)
						{																									// <=> 2 seuils de menures mais pour la valeur du seuil et non pour l'intervaleur
							if (srt[lc_i][P_MSR] == srt[lc_i][P_IMN])
								srt[lc_i][P_CMM] = srt[lc_i][P_OMN];
							else
								srt[lc_i][P_CMM] = !srt[lc_i][P_OMN];
						}
						else																								// deux seuil de sortie (mini et maxi)
						{
							if (srt[lc_i][P_MSR] < srt[lc_i][P_IMN])
								srt[lc_i][P_CMM] = srt[lc_i][P_OMN];
							if (srt[lc_i][P_MSR] > srt[lc_i][P_IMX])
								srt[lc_i][P_CMM] = srt[lc_i][P_OMX];
						}
					}
					else																								// seuil mesure on!=off OU off!=on
					{
						if (srt[lc_i][P_OMN] == srt[lc_i][P_OMX])																// un seul seuil de sortie (mini == maxi)
						{
							if ((srt[lc_i][P_MSR] > srt[lc_i][P_IMN]) && (srt[lc_i][P_MSR] < srt[lc_i][P_IMX]))
								srt[lc_i][P_CMM] = srt[lc_i][P_OMN];
							if ((srt[lc_i][P_MSR] < srt[lc_i][P_IMN]) || (srt[lc_i][P_MSR] > srt[lc_i][P_IMX]))
								srt[lc_i][P_CMM] = !srt[lc_i][P_OMN];
						}
						else																								// deux seuil de sortie (mini et maxi)
						{
							if (srt[lc_i][P_IMN] == srt[lc_i][P_OMN])														// ordre identique on.off et on.off OU off.on et off.on
							{
								if (srt[lc_i][P_MSR] == srt[lc_i][P_OMN])
									srt[lc_i][P_CMM] = srt[lc_i][P_OMN];
								else
									srt[lc_i][P_CMM] = srt[lc_i][P_OMX];
							}
							else																						// ordre inverse on.off et off.on OU off.on et on.off
							{
								if (srt[lc_i][P_MSR] == srt[lc_i][P_OMN])
									srt[lc_i][P_CMM] = srt[lc_i][P_OMX];
								else
									srt[lc_i][P_CMM] = srt[lc_i][P_OMN];
							}
						}
					}
				}
				
				if (srt[lc_i][P_CMM] == 0)
				{
					digitalWrite(srt[lc_i][P_OBR], OFF);
					fv_send(5, lc_i, P_CMM, gr_prt[6], 0);
				}
				else
				{
					digitalWrite(srt[lc_i][P_OBR], ON);
					fv_send(5, lc_i, P_CMM, gr_prt[5], 0);
				}
			}
		}
	}
	
	return(0);
}

//
// lg longueur send
// pr parametre a envoyer (MSR, CMM...)
// ns numero de la sortie
// v1 valeur a envoyer (1c)
// v2 valeur a envoyer (uniquement si 2c a envoyer)
//

void fv_send(char lg, char ns, char pr, unsigned char v1, unsigned char v2)
{
	if (digitalRead(B_DSR))																								// !DSR
		return;
		
	Serial.write(&gr_prt[0], 1);
	Serial.write(&gr_lng[lg], 1);
	Serial.write(gr_ga, 3);
	if (lg != 1)
	{
		Serial.write(&gr_prt[4], 1);
		Serial.write(&gr_prt[7], 1);
		Serial.write(&gr_lng[ns], 1);
		Serial.write(&gr_lng[pr], 1);
		Serial.write(&v1, 1);
		if (lg == 6)
			Serial.write(&v2, 1);
	}
	else
		Serial.write(&gr_prt[ns], 1);
	Serial.write(&gr_prt[1], 1);
}

// En fin de loop()

void serialEvent(void) 
{
	char *p;
		
	if (digitalRead(B_DSR))																								// !DSR
		return;

	if ((gc_f == 2) && (gc_r != 0))																						// rcv et wrk sont plein pas reception
		return;
		
	if ((gc_f == 2) && (gc_r == 0))
	{
		strncpy(wrk, &rcv[5], strlen(rcv)-6);
		wrk[strlen(rcv)-6] = 0;
		gc_x = 0;
		gc_r = 1;
		gc_f = 0;
	}

	while (Serial.available()) 
	{
		rcv[gc_x] = (char) Serial.read(); 
		if (rcv[gc_x] == '\r')
			continue;

		if (gc_x == 0)
		{
			if (rcv[0] == '\t')																							// debut commande
				gc_f = 1;
		}
		else
		{
			if (rcv[gc_x] == '\n')																						// fin commande
			{
				rcv[gc_x+1] = 0;
				if ((strlen(rcv)-6) == (rcv[1]-'a'))
					gc_f = 2;
				else
				{
					gc_x = 0;
					gc_f = 0;
					fv_send(1, PRT_NACK, 0, 0, 0);
				}
				break;
			}
		}
		
		if (gc_f == 1)
		{
			gc_x++;
			if (gc_x >= LNG)																							// buffer plein --> efface (pas d'erreur)
			{
				gc_x = 0;
				gc_f = 0;
				fv_send(1, PRT_NACK, 0, 0, 0);
			}
		}
	}
		
	if ((gc_f == 2) && (gc_r == 0))
	{
		strncpy(wrk, &rcv[5], strlen(rcv)-6);
		wrk[strlen(rcv)-6] = 0;
		gc_x = 0;
		gc_r = 1;
		gc_f = 0;
	}
}

