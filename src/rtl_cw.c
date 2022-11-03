/*
 * rtl-sdr, turns your Realtek RTL2832 based DVB dongle into a SDR receiver
 * Copyright (C) 2012 by Steve Markgraf <steve@steve-m.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#ifndef _WIN32
#include <unistd.h>
#else
#include <windows.h>
#include <io.h>
#include <fcntl.h>
#include "getopt/getopt.h"
#endif

#include "rtl-sdr.h"
#include "convenience/convenience.h"

#define DEFAULT_SAMPLE_RATE		2400000
//#define DEFAULT_BUF_LENGTH	(16 * 16384)
#define DEFAULT_BUF_LENGTH		(16384)
#define MINIMAL_BUF_LENGTH		512
#define MAXIMAL_BUF_LENGTH		(256 * 16384)


#define INIT_FLTR (2) //first filter order: order Number of terms 1 more than the order

#define GAUSS_TAPS 	(200) //size of gaussian filter
#define TRUE 		(1) 
#define FALSE 		(0) 

#define FRST_FLTR (8) //first filter 7th order: order Number of terms 1 more than the order
#define denomOne	(8254409552.019)
#define consta 		(54867539664.451/denomOne) 
#define constb 		(156373549966.423/denomOne) 
#define constc 		(247701763446.204/denomOne) 
#define constd 		(235522413677.033/denomOne) 
#define conste 		(134421433690.466/denomOne) 
#define constf 		(42639309193.356/denomOne) 
#define constg 		(5798945459.710/denomOne) 

#define denomHP	(1.193)
#define a 		(7.930/denomHP) 
#define b 		(22.602/denomHP) 
#define c 		(35.802/denomHP) 
#define d 		(34.042/denomHP) 
#define e 		(19.429/denomHP) 
#define f 		(6.163/denomHP) 
#define g 		(0.838/denomHP) 



struct llist {
	char *data;
	size_t len;
	struct llist *next;
};

static int do_exit = 0;
static uint32_t bytes_to_read = 0;
static rtlsdr_dev_t *dev = NULL;

#define GAUSS_TAPS 	(200) //size of gaussian filter
#define TRUE 		(1) 
#define FALSE 		(0) 



float gaussian[] = {
0.0000276867,
0.0000313372,
0.0000354252,
0.0000399970,
0.0000451024,
0.0000507969,
0.0000571393,
0.0000641942,
0.0000720305,
0.0000807234,
0.0000903534,
0.0001010067,
0.0001127759,
0.0001257607,
0.0001400663,
0.0001558062,
0.0001730996,
0.0001920743,
0.0002128643,
0.0002356122,
0.0002604675,
0.0002875883,
0.0003171365,
0.0003492890,
0.0003842253,
0.0004221278,
0.0004631981,
0.0005076294,
0.0005556344,
0.0006074231,
0.0006632135,
0.0007232295,
0.0007877002,
0.0008568493,
0.0009309116,
0.0010101191,
0.0010947065,
0.0011849031,
0.0012809352,
0.0013830323,
0.0014914123,
0.0016062852,
0.0017278555,
0.0018563163,
0.0019918500,
0.0021346223,
0.0022847824,
0.0024424682,
0.0026077876,
0.0027808234,
0.0029616639,
0.0031503478,
0.0033469028,
0.0035512460,
0.0037634049,
0.0039832968,
0.0042107560,
0.0044456995,
0.0046878789,
0.0049371008,
0.0051931166,
0.0054555949,
0.0057242044,
0.0059985860,
0.0062782701,
0.0065628150,
0.0068516959,
0.0071443880,
0.0074403389,
0.0077388583,
0.0080393386,
0.0083410893,
0.0086433647,
0.0089454468,
0.0092465623,
0.0095458826,
0.0098426069,
0.0101359066,
0.0104249255,
0.0107088076,
0.0109867242,
0.0112577916,
0.0115211814,
0.0117760096,
0.0120214479,
0.0122566952,
0.0124809783,
0.0126934963,
0.0128935035,
0.0130803370,
0.0132533065,
0.0134118320,
0.0135552782,
0.0136831756,
0.0137950272,
0.0138904462,
0.0139690461,
0.0140305507,
0.0140747113,
0.0141013900,
0.0141104762,
0.0141019147,
0.0140757608,
0.0140320973,
0.0139710898,
0.0138929870,
0.0137980375,
0.0136866002,
0.0135590894,
0.0134160022,
0.0132578082,
0.0130851149,
0.0128985023,
0.0126986608,
0.0124862809,
0.0122620807,
0.0120268609,
0.0117813951,
0.0115264840,
0.0112630114,
0.0109917782,
0.0107136406,
0.0104294824,
0.0101401321,
0.0098465010,
0.0095493901,
0.0092496279,
0.0089480981,
0.0086455189,
0.0083427464,
0.0080404709,
0.0077394383,
0.0074403665,
0.0071438908,
0.0068506188,
0.0065611580,
0.0062760607,
0.0059958242,
0.0057209179,
0.0054517837,
0.0051887806,
0.0049322953,
0.0046826039,
0.0044399827,
0.0042046249,
0.0039767791,
0.0037565557,
0.0035440930,
0.0033394184,
0.0031426425,
0.0029537376,
0.0027727038,
0.0025995161,
0.0024340780,
0.0022763065,
0.0021260912,
0.0019832968,
0.0018477686,
0.0017193409,
0.0015978315,
0.0014830441,
0.0013747746,
0.0012728101,
0.0011769243,
0.0010868990,
0.0010024966,
0.0009234880,
0.0008496356,
0.0007807074,
0.0007164715,
0.0006566930,
0.0006011456,
0.0005496055,
0.0005018518,
0.0004576718,
0.0004168556,
0.0003792044,
0.0003445166,
0.0003126100,
0.0002833021,
0.0002564185,
0.0002317937,
0.0002092704,
0.0001886977,
0.0001699335,
0.0001528425,
0.0001372977,
0.0001231784,
0.0001103721,
0.0000987724,
0.0000882807,
0.0000788043,
0.0000702563,
0.0000625568,
0.0000556308,
0.0000494094,
0.0000438284,
0.0000388288,
0.0000343561,
0.0000303607
};


void usage(void)
{
	fprintf(stderr,
		"rtl_sdr, an I/Q recorder for RTL2832 based DVB-T receivers\n\n"
		"Usage:\t -f frequency_to_tune_to [Hz]\n"
		"\t[-d device_index (default: 0)]\n"
		"\t[-g gain (default: 0 for auto)]\n"
		"\t[-p ppm_error (default: 0)]\n"
		"\t[-n number of samples to read (default: 0, infinite)]\n"
		"\t[-r gain (db) (invokes resonant filter stage)]\n"
		"\t[-w WPM (resampling rate for binary output)]\n"
		"\t[-t threshold trigger level]\n"
		"\t[-l lower threshold level]\n"
		"\tfilename (if no file name defaults to stdout)\n\n");
	exit(1);
}

#ifdef _WIN32
BOOL WINAPI
sighandler(int signum)
{
	if (CTRL_C_EVENT == signum) {
		fprintf(stderr, "Signal caught, exiting!\n");
		do_exit = 1;
		rtlsdr_cancel_async(dev);
		return TRUE;
	}
	return FALSE;
}
#else
static void sighandler(int signum)
{
	fprintf(stderr, "Signal caught, exiting!\n");
	do_exit = 1;
	rtlsdr_cancel_async(dev);
}
#endif

	int sockfda,sockfdb;
	struct addrinfo hintsa,hintsb, *servinfoa, *servinfob, *porta,*portb;
	int rva,rvb;


	float x[INIT_FLTR], y[INIT_FLTR], u[INIT_FLTR], v[INIT_FLTR];
	double lpx[FRST_FLTR], lpy[FRST_FLTR], lpu[FRST_FLTR], lpv[FRST_FLTR];
	double hpx[FRST_FLTR], hpy[FRST_FLTR], hpu[FRST_FLTR], hpv[FRST_FLTR];
	
	int		i = 0, j = 0, k = 0, l = 0, m = 0, ii = 0, jj = 0, kk = 0, ll = 0, mm = 0, nn = 0, oo = 0, pp = 0, qq = 0, kdiv = 0;

	double 	attenuate = 0.01;
	float 	Damper = 1.0;

	double 	Omega = .1309 ;
	double 	Damping = .014 ; // 0.00455 	
	double 	Centre_Damping = .010 ; // 0.00455 	

	double	integ_0_in, integ_1_in, integ_2_in, integ_3_in, integ_4_in, integ_5_in, integ_6_in, integ_7_in;
	double	integ_0_out = 0.0, integ_1_out = 0.0, integ_2_out = 0.0, integ_3_out = 0.0, integ_4_out = 0.0, integ_5_out = 0.0, integ_6_out = 0.0, integ_7_out = 0.54; // ~500Hz

	double	lug_0 = 0, lug_1 = 0, lug_2 = 0, lug_3 = 0, lug_4 = 0, lug_5 = 0, lug_6 = 0, lug_7 = 0; 


	int 	resampleFlag = FALSE, iqFlag = FALSE, hystFlag = FALSE; 
	float 	resample = 0.0; // allowing for variable rates of sampling
	float	wpm = 25.0; //default rate
	float   resamplestep;
	float	threshold = 0.5, hysteresis = 0.0; // default threshold for coarse graining .5 ... drops out at .5 -.1
	int 	resonantFilterFlag=FALSE;

	int 	dataIndx = 0;
	float 	gaussA = 0.0, gaussB = 0.0 ;
	float 	dataA[GAUSS_TAPS], dataB[GAUSS_TAPS];
	int 	flag = FALSE;
	int numbytes, hostipFlag = FALSE, hostipCount = 0;
	char hostip[20];
	long int counter = 0;


static void rtlsdr_callback(unsigned char *buf, uint32_t len, void *ctx)
{
	struct llist *rpt;
	
    float float_IQ[2];
	char message[50];

	int 	count;

	if (ctx) {
		if (do_exit)
			return;

		if ((bytes_to_read > 0) && (bytes_to_read < len)) {
			len = bytes_to_read;
			do_exit = 1;
			rtlsdr_cancel_async(dev);
		}

		rpt = (struct llist*)malloc(sizeof(struct llist));
		rpt->data = (char*) malloc(len);
		memcpy(rpt->data, buf, len);
		rpt->len = len;
		rpt->next = NULL;

		i=0;
		count = 0;
		resamplestep = wpm/25.0;
		
		while (count < (int) rpt->len) {
			i = (i + 1) % INIT_FLTR;
			j = (i + 1) % INIT_FLTR;
			x[i] = (float)rpt->data[count++]  - 127.5; //I
			u[i] = (float)rpt->data[count++]  - 127.5; //Q

			// 1274.239⋅yi = (xi + 1⋅xi-1) + 1272.239⋅yi-1	(600Hz cut off)
			y[i] = (x[i] + x[j])/163102.592 + 0.99843043573458354358*y[j];	
			v[i] = (u[i] + u[j])/163102.592 + 0.99843043573458354358*v[j];	

	// move two gaussian calculations on one step each loop, to spread the computational load
			if (flag == TRUE){
				//fprintf(stderr, "%d: %d;  %d %f\n",flag, m, l, gaussian[l]);
				gaussA = gaussA + dataA[m]*gaussian[l];
				gaussB = gaussB + dataB[m]*gaussian[l];
				l = l + 1  ;
				m = (m + 1) % GAUSS_TAPS;
				if (l >= GAUSS_TAPS ) {flag = FALSE; l=0;}
			}

			k++; 
			if (k == 50) { // downsample to 48kHz

				pp = ii;					// ii -1
				ii = (ii + 1) % FRST_FLTR;	// ii
				jj = (ii + 1) % FRST_FLTR;	// ii -6
				kk = (jj + 1) % FRST_FLTR;	// ii -5
				ll = (kk + 1) % FRST_FLTR;	// ii -4
				mm = (ll + 1) % FRST_FLTR;	// ii -3
				nn = (mm + 1) % FRST_FLTR;	// ii -2
				oo = (nn + 1) % FRST_FLTR;	// ii -2


				lpx[ii] = (double) y[i]; //input to I
				lpu[ii] = (double) v[i]; //input to Q

				hpx[ii] = lpy[ii] = (lpx[ii] + 7*lpx[pp] + 21*lpx[oo] + 35*lpx[nn] + 35*lpx[mm] + 21*lpx[ll] + 7*lpx[kk] + lpx[jj])/denomOne
					+ consta*lpy[pp] - constb*lpy[oo] + constc*lpy[nn] - constd*lpy[mm] + conste*lpy[ll] - constf*lpy[kk] + constg*lpy[jj]; 


				hpu[ii] = lpv[ii] = (lpu[ii] + 7*lpu[pp] + 21*lpu[oo] + 35*lpu[nn] + 35*lpu[mm] + 21*lpu[ll] + 7*lpu[kk] + lpu[jj])/denomOne
					+ consta*lpv[pp] - constb*lpv[oo] + constc*lpv[nn] - constd*lpv[mm] + conste*lpv[ll] - constf*lpv[kk] + constg*lpv[jj]; 



				// 1.057⋅yi = (1⋅xi + -2⋅xi-1 + 1⋅xi-2) + 1.997⋅yi-1 + -0.946⋅yi-2
				hpy[ii] = (hpx[ii] - 2*hpx[pp] + hpx[oo])/1.057 + 1.88930936613055818353*hpy[pp] - .89498580889309366130*hpy[oo];	
				hpv[ii] = (hpu[ii] - 2*hpu[pp] + hpu[oo])/1.057 + 1.88930936613055818353*hpv[pp] - .89498580889309366130*hpv[oo];	

		// transposed from pertecs ...  auto-centered tuning with feed back from side-tuned resonant filters

				lug_0 = integ_0_out*integ_0_out + integ_1_out*integ_1_out;
				lug_2 = integ_4_out*integ_4_out + integ_5_out*integ_5_out;
				lug_3 = integ_7_out -.02;
				lug_4 = integ_7_out;
				lug_5 = integ_7_out +.02;
				lug_6 = lug_0-lug_2;
				lug_7 = lug_2 + lug_0;

		// update input values
				integ_0_in = integ_2_in = integ_4_in =  attenuate*hpy[ii];
				integ_1_in = integ_3_in = integ_5_in =  attenuate*hpv[ii]; 

				integ_0_in = integ_0_in + lug_3 * (Omega * integ_1_out - Damping * integ_0_out);
				integ_1_in = integ_1_in - lug_3 * (Omega * integ_0_out + Damping * integ_1_out);

				integ_2_in = integ_2_in + lug_4 * (Omega * integ_3_out - Damper*Damping * integ_2_out);		//main pair
				integ_3_in = integ_3_in - lug_4 * (Omega * integ_2_out + Damper*Damping * integ_3_out);


				integ_4_in = integ_4_in + lug_5 * (Omega * integ_5_out - Damping * integ_4_out);
				integ_5_in = integ_5_in - lug_5 * (Omega * integ_4_out + Damping * integ_5_out);
				// feedback on tuning
				integ_6_in = .15e-5*lug_6*lug_7 - 2e-3 * integ_6_out;
				integ_7_in = -1.0e-5*integ_6_out - 0.5e-8  + 1e-8*integ_7_out;
		
		

		// update output values
				integ_0_out += integ_0_in;
				integ_1_out += integ_1_in;

				integ_2_out += integ_2_in;
				integ_3_out += integ_3_in;

				integ_4_out += integ_4_in;
				integ_5_out += integ_5_in;

				integ_6_out += integ_6_in;
				integ_7_out += integ_7_in;

		 // end of pertecs trnaspose

				kdiv++; 
				if (kdiv == 10) { //drop another factor of ten to 4800Hz resample for gaussian filters 

					// LOAD NEW VALUE TO INPUT GAUSSIAN FILTER
					dataA[dataIndx] = (hpy[ii]*hpy[ii] + hpv[ii]*hpv[ii]); 
					dataB[dataIndx] = (integ_2_out*integ_2_out + integ_3_out*integ_3_out);

					// OUTPUT 
					// output gaussian filtered data

						if (threshold == 0.0) {
							sprintf(message, "%ld %.8f %.8f",counter , float_IQ[0] = (float) 3*logf(1+  100.0 * gaussA), (float)  logf(1+ 100.0 * gaussB));
						}
						else if (resonantFilterFlag==FALSE) {
							sprintf(message, "%ld %.8f %.5f %.5f",counter , float_IQ[0] = (float)  3*logf(1+  100.0 * gaussA), (float)  threshold, (float)  threshold - hysteresis);
						}
						else if (resonantFilterFlag==TRUE) { 
							sprintf(message, "%ld %.8g %.5g %.5g",counter , float_IQ[0] = (float)  3*logf(1+  100.0 * gaussB), (float)  threshold, (float)  threshold - hysteresis);
						}
						// send udp
				
						if (hostipFlag == TRUE) {
							hostipCount = (hostipCount + 1) % 10;
							if (hostipCount == 0) {
								counter = (counter+1) % 10000000; 
								if ((numbytes = sendto(sockfda, &message[0], strlen(message), 0,porta->ai_addr, porta->ai_addrlen)) == -1) {
									perror("Display UDP error");
									exit (1);
								}
							}
					
						}

					if (iqFlag == FALSE) { //not outputting raw I/Q data
//**********************
						if (resampleFlag==TRUE) {
							resample += resamplestep;
					
							if (hystFlag == FALSE) {
								if (float_IQ[0] > threshold) { // if the threshold is met in the lead up then relax the sampling threshold
										hystFlag = TRUE;
								}
							} 
							if (resample >=120){ // 25wpm if resamplestep = wpm/25.0
								// if level > lowered threshold
								if (float_IQ[0] >= threshold - hysteresis) {
									sprintf(message, "1");
									// fprintf((FILE*)ctx, "1\n");
								}
								else {
									sprintf(message, "0");
									// fprintf((FILE*)ctx, "0\n");
									hystFlag = FALSE;
								}
								if ((numbytes = sendto(sockfdb, &message[0], strlen(message), 0,portb->ai_addr, portb->ai_addrlen)) == -1) {
									perror("Binary stream error");
									exit (1);
								}
								//fflush((FILE*)ctx);
								resample = 0;
							}
						} 
//**********************
		//
		//				else {// resampleFlag == FALSE
		//					fwrite(&float_IQ[0], sizeof(float), 2, (FILE*)ctx); 
		//				}		
					} else { //output I/Q data only
						if (resonantFilterFlag==FALSE) {
							float_IQ[0] = (float) hpy[ii]; 
							float_IQ[1] = (float) hpv[ii];
						} else {
							float_IQ[0] = (float) integ_2_out; 
							float_IQ[1] = (float) integ_3_out;				
						} 
						fwrite(&float_IQ[0], sizeof(float), 2, (FILE*)ctx); 

					}
					gaussA = gaussB = 0.0;
					m = dataIndx = (dataIndx + 1) % GAUSS_TAPS; // increment the data index
					l = 0; 
					flag = TRUE;

//					float_IQ[0] = (float) integ_2_out;
//					float_IQ[1] = (float) integ_3_out;

//					fwrite(float_IQ, sizeof(float), 2, (FILE*)ctx);




					kdiv = 0; 
				}
				k=0;
			}
		}
		free(rpt->data);
		free(rpt);

//		if (fwrite(buf, 1, len, (FILE*)ctx) != len) {
//			fprintf(stderr, "Short write, samples lost, exiting!\n");
//			rtlsdr_cancel_async(dev);
//		}

		if (bytes_to_read > 0)
			bytes_to_read -= len;
	}
}

int main(int argc, char **argv)
{
#ifndef _WIN32
	struct sigaction sigact;
#endif
	char *filename = NULL;
	int n_read;
	int r, opt;
	int gain = 0;
	int ppm_error = 0;
	int sync_mode = 0;
	FILE *file;
	uint8_t *buffer;
	int dev_index = 0;
	int dev_given = 0;
	uint32_t frequency = 27085550;
	uint32_t samp_rate = DEFAULT_SAMPLE_RATE;
	uint32_t out_block_size = DEFAULT_BUF_LENGTH;

	strcpy(&hostip[0], "127.0.0.1"); /// local host is the default for the decoding. 

//	while ((opt = getopt(argc, argv, "d:f:g:s:b:n:p:S")) != -1) {
	while ((opt = getopt(argc, argv, "d:f:g:n:p:r:w:t:l:h:")) != -1) {
		switch (opt) {
		case 'd':
			dev_index = verbose_device_search(optarg);
			dev_given = 1;
			break;
		case 'f':
			frequency = (uint32_t)atofs(optarg) + 550; // always add 550 to get the CW offset
			break;
		case 'g':
			gain = (int)(atof(optarg) * 10); /* tenths of a dB */
			break;
		case 'r': // includes the input gain for resonant filter stage
			resonantFilterFlag = TRUE;
			attenuate = exp(((float) (atof(optarg)))*0.230258509)*.001; // .001 is the initial default coupling gain
			break;
		case 'p':
			ppm_error = atoi(optarg);
			break;
		case 'n':
			bytes_to_read = (uint32_t)atof(optarg) * 2;
			break;
		case 'w': // takes a wpm argument
			resampleFlag=TRUE;
			resamplestep = (float) (atof(optarg))/25.0;
			break;
		case 't': //threshold top level
			threshold = (float) (atof(optarg)); //default 0.5 
			break;
		case 'l': // threshold lower level
			hysteresis = (float) (atof(optarg)); // 
			if (hysteresis <= 0) hysteresis = .2*threshold;
			else if (hysteresis > threshold) hysteresis = 0.5*threshold;
			break;
		case 'h': // host ip
			hostipFlag = TRUE;
			hostipCount = 0;
			strcpy(&hostip[0], optarg);
			;
			break;
		default:
			usage();
			break;
		}
	}

	if (argc <= optind) {
		usage();
	} else {
		filename = argv[optind];
	}

//*********************** networked data display and binary stream
	memset(&hintsa, 0, sizeof hintsa);
	hintsa.ai_family = AF_UNSPEC;
	hintsa.ai_socktype = SOCK_DGRAM;

	memset(&hintsb, 0, sizeof hintsb);
	hintsb.ai_family = AF_UNSPEC;
	hintsb.ai_socktype = SOCK_DGRAM;


	if (hostipFlag == TRUE) { // services the scope 
// first port for scope 9224
		if ((rva = getaddrinfo(&hostip[0], "9224", &hintsa, &servinfoa)) != 0) {
			fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rva));
			return 1;
		}

	// loop through all the results and make a socket
		for(porta = servinfoa; porta != NULL; porta = porta->ai_next) {
			if ((sockfda = socket(porta->ai_family, porta->ai_socktype,
					porta->ai_protocol)) == -1) {
				perror("udpsend: socket 9224");
				continue;
			}

			break;
		}

		if (porta == NULL) {
			fprintf(stderr, "udpsend: failed to bind socket 9224\n");
			return 2;
		}
	}
//second port 9222 is always set up 
	if ((rvb = getaddrinfo(&hostip[0], "9222", &hintsb, &servinfob)) != 0) {
		fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rvb));
		return 1;
	}

// loop through all the results and make a socket
	for(portb = servinfob; portb != NULL; portb = portb->ai_next) {
		if ((sockfdb = socket(portb->ai_family, portb->ai_socktype,
				portb->ai_protocol)) == -1) {
			perror("udpsend: socket 9222");
			continue;
		}

		break;
	}

	if (portb == NULL) {
		fprintf(stderr, "udpsend: failed to bind socket 9222\n");
		return 2;
	}


//***********************



	if(out_block_size < MINIMAL_BUF_LENGTH ||
	   out_block_size > MAXIMAL_BUF_LENGTH ){
		fprintf(stderr,
			"Output block size wrong value, falling back to default\n");
		fprintf(stderr,
			"Minimal length: %u\n", MINIMAL_BUF_LENGTH);
		fprintf(stderr,
			"Maximal length: %u\n", MAXIMAL_BUF_LENGTH);
		out_block_size = DEFAULT_BUF_LENGTH;
	}

	buffer = malloc(out_block_size * sizeof(uint8_t));

	if (!dev_given) {
		dev_index = verbose_device_search("0");
	}

	if (dev_index < 0) {
		exit(1);
	}

	r = rtlsdr_open(&dev, (uint32_t)dev_index);
	if (r < 0) {
		fprintf(stderr, "Failed to open rtlsdr device #%d.\n", dev_index);
		exit(1);
	}
#ifndef _WIN32
	sigact.sa_handler = sighandler;
	sigemptyset(&sigact.sa_mask);
	sigact.sa_flags = 0;
	sigaction(SIGINT, &sigact, NULL);
	sigaction(SIGTERM, &sigact, NULL);
	sigaction(SIGQUIT, &sigact, NULL);
	sigaction(SIGPIPE, &sigact, NULL);
#else
	SetConsoleCtrlHandler( (PHANDLER_ROUTINE) sighandler, TRUE );
#endif
	/* Set the sample rate */
	verbose_set_sample_rate(dev, samp_rate);

	/* Set the frequency */
	verbose_set_frequency(dev, frequency);

	if (0 == gain) {
		 /* Enable automatic gain */
		verbose_auto_gain(dev);
	} else {
		/* Enable manual gain */
		gain = nearest_gain(dev, gain);
		verbose_gain_set(dev, gain);
	}

	verbose_ppm_set(dev, ppm_error);

	if(strcmp(filename, "-") == 0) { /* Write samples to stdout */
		file = stdout;
#ifdef _WIN32
		_setmode(_fileno(stdin), _O_BINARY);
#endif
	} else {
		file = fopen(filename, "wb");
		if (!file) {
			fprintf(stderr, "Failed to open %s\n", filename);
			goto out;
		}
	}


	for (i=0; i < INIT_FLTR; i++) {
		x[i] = y[i] = u[i] = v[i] = 0.0;
	}
	for (ii=0; ii < FRST_FLTR; ii++) {
		lpx[ii] = lpy[ii] = lpu[ii] = lpv[ii] = 0.0;
		hpx[ii] = hpy[ii] = hpu[ii] = hpv[ii] = 0.0;
	}



	/* Reset endpoint before we start reading from it (mandatory) */
	verbose_reset_buffer(dev);

/*	if (sync_mode) {
		fprintf(stderr, "Reading samples in sync mode...\n");
		while (!do_exit) {
			r = rtlsdr_read_sync(dev, buffer, out_block_size, &n_read);
			if (r < 0) {
				fprintf(stderr, "WARNING: sync read failed.\n");
				break;
			}

			if ((bytes_to_read > 0) && (bytes_to_read < (uint32_t)n_read)) {
				n_read = bytes_to_read;
				do_exit = 1;
			}

			if (fwrite(buffer, 1, n_read, file) != (size_t)n_read) {
				fprintf(stderr, "Short write, samples lost, exiting!\n");
				break;
			}

			if ((uint32_t)n_read < out_block_size) {
				fprintf(stderr, "Short read, samples lost, exiting!\n");
				break;
			}

			if (bytes_to_read > 0)
				bytes_to_read -= n_read;
		}
	} else { */
		fprintf(stderr, "Reading samples in async mode...\n");
		r = rtlsdr_read_async(dev, rtlsdr_callback, (void *)file,
				      0, out_block_size);
/*	} */

	if (do_exit)
		fprintf(stderr, "\nUser cancel, exiting...\n");
	else
		fprintf(stderr, "\nLibrary error %d, exiting...\n", r);

	if (file != stdout)
		fclose(file);

	rtlsdr_close(dev);
	free (buffer);
	freeaddrinfo(servinfoa);
	freeaddrinfo(servinfob);
	close(sockfda);
	close(sockfdb);

	
	
out:
	return r >= 0 ? r : -r;
}
