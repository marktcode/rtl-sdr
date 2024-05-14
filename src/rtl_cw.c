/* This is not rtl_fm but rtl_cw parading for compilation
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
 
 ./rtl_cw -f 7.01100e6 -a 5 -h 192.168.1.7 -w 25 -q - | spectrum3 -h 192.168.1.7 -r 10 -g 0 

./rtl_cw -f 7.01100e6 -a 2 -h 192.168.1.7 -w 25 -q -   | spectrum3 -h 192.168.1.7 -v 2

 echo "-f 27.00004e6 -r 8" | framesnd 192.168.1.111 9223 > /dev/null
 
 on the mac
 cd /Volumes/pi
framerec 9224 | pertecs -c viewiq -rate 48000 &
framerec 9226 | pertecs -c displayspectrum -rate 100000 &
 framerec 9224 | pertecs -c viewiq -rate 48000 &
 
 cd /Volumes/pi/morsedemod
 ./tc_mrsdcdr_mac
 
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
#include <pthread.h>
#include <wordexp.h>

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

#define MAXBUFLEN 1024 /// string processing

#define INIT_FLTR (2) //first filter order: order Number of terms 1 more than the order

#define GAUSS_TAPS 	(200) //size of gaussian filter
#define	DIFFERENTIAL_TAPS	(31)	// size of the differential filter
#define DEFAULT_RESPONSE (.0003) // DC-removam response parameter

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

#define defaultfreq 	(27085000) 
#define freqoffset 		(640) 

#define MARK_freq		(0.610)
#define SPACE_freq		(0.485)

#define LO				(800) // local oscillator set to 800Hz


#define HDAMP	0.007 //.0068
#define LDAMP	0.007


struct llist {
	char *data;
	size_t len;
	struct llist *next;
};

static int do_exit = 0;
static uint32_t bytes_to_read = 0;
static rtlsdr_dev_t *dev = NULL;

static float mixer, t = 0;


struct control {
	int		In_flg;
	int		direct_sampling;
    int 	enable_biastee;
	int		agcFlag;
	uint32_t frequency;
	float	increment;			// sets local oscillator rate 
	int		testsignal;
	float	signalVnoise;
	int		cwFlg; // TRUE for CW decoding 
	float	localosc;			// sets local oscillator rate 
	float	localoscsv;
	int 	LNAgain;
	int		resonantFilterFlag;
	float 	attenuate;
	float 	balance;
	float 	damper;
	int		resampleFlag;
	float	WordsPerMinute;
	float	threshold; //top threshold
	float	response;
	float	ScaleSpectrum;
	int		Streaming_Flg; /// normally on for streaming of signal and spectrum 
};

int	rc;
struct control	parameters;
pthread_t 	controls_thread;


float gaussian[] = {
0.0000276867, 0.0000313372, 0.0000354252, 0.0000399970, 0.0000451024, 0.0000507969, 0.0000571393, 0.0000641942, 0.0000720305, 0.0000807234,
0.0000903534, 0.0001010067, 0.0001127759, 0.0001257607, 0.0001400663, 0.0001558062, 0.0001730996, 0.0001920743, 0.0002128643, 0.0002356122,
0.0002604675, 0.0002875883, 0.0003171365, 0.0003492890, 0.0003842253, 0.0004221278, 0.0004631981, 0.0005076294, 0.0005556344, 0.0006074231,
0.0006632135, 0.0007232295, 0.0007877002, 0.0008568493, 0.0009309116, 0.0010101191, 0.0010947065, 0.0011849031, 0.0012809352, 0.0013830323,
0.0014914123, 0.0016062852, 0.0017278555, 0.0018563163, 0.0019918500, 0.0021346223, 0.0022847824, 0.0024424682, 0.0026077876, 0.0027808234,
0.0029616639, 0.0031503478, 0.0033469028, 0.0035512460, 0.0037634049, 0.0039832968, 0.0042107560, 0.0044456995, 0.0046878789, 0.0049371008,
0.0051931166, 0.0054555949, 0.0057242044, 0.0059985860, 0.0062782701, 0.0065628150, 0.0068516959, 0.0071443880, 0.0074403389, 0.0077388583,
0.0080393386, 0.0083410893, 0.0086433647, 0.0089454468, 0.0092465623, 0.0095458826, 0.0098426069, 0.0101359066, 0.0104249255, 0.0107088076,
0.0109867242, 0.0112577916, 0.0115211814, 0.0117760096, 0.0120214479, 0.0122566952, 0.0124809783, 0.0126934963, 0.0128935035, 0.0130803370,
0.0132533065, 0.0134118320, 0.0135552782, 0.0136831756, 0.0137950272, 0.0138904462, 0.0139690461, 0.0140305507, 0.0140747113, 0.0141013900,
0.0141104762, 0.0141019147, 0.0140757608, 0.0140320973, 0.0139710898, 0.0138929870, 0.0137980375, 0.0136866002, 0.0135590894, 0.0134160022,
0.0132578082, 0.0130851149, 0.0128985023, 0.0126986608, 0.0124862809, 0.0122620807, 0.0120268609, 0.0117813951, 0.0115264840, 0.0112630114,
0.0109917782, 0.0107136406, 0.0104294824, 0.0101401321, 0.0098465010, 0.0095493901, 0.0092496279, 0.0089480981, 0.0086455189, 0.0083427464,
0.0080404709, 0.0077394383, 0.0074403665, 0.0071438908, 0.0068506188, 0.0065611580, 0.0062760607, 0.0059958242, 0.0057209179, 0.0054517837,
0.0051887806, 0.0049322953, 0.0046826039, 0.0044399827, 0.0042046249, 0.0039767791, 0.0037565557, 0.0035440930, 0.0033394184, 0.0031426425,
0.0029537376, 0.0027727038, 0.0025995161, 0.0024340780, 0.0022763065, 0.0021260912, 0.0019832968, 0.0018477686, 0.0017193409, 0.0015978315,
0.0014830441, 0.0013747746, 0.0012728101, 0.0011769243, 0.0010868990, 0.0010024966, 0.0009234880, 0.0008496356, 0.0007807074, 0.0007164715,
0.0006566930, 0.0006011456, 0.0005496055, 0.0005018518, 0.0004576718, 0.0004168556, 0.0003792044, 0.0003445166, 0.0003126100, 0.0002833021,
0.0002564185, 0.0002317937, 0.0002092704, 0.0001886977, 0.0001699335, 0.0001528425, 0.0001372977, 0.0001231784, 0.0001103721, 0.0000987724,
0.0000882807, 0.0000788043, 0.0000702563, 0.0000625568, 0.0000556308, 0.0000494094, 0.0000438284, 0.0000388288, 0.0000343561, 0.0000303607
};

float zcoef15[] = {-4.105571847507337e-02, -2.639296187683288e-02, -1.274142987157447e-02, -1.011224592982157e-04, 1.152796035999595e-02, 
2.214581858630802e-02, 3.175245221963798e-02, 4.034786125998584e-02, 4.793204570735161e-02, 5.450500556173526e-02, 6.006674082313684e-02, 
6.461725149155630e-02, 6.815653756699364e-02, 7.068459904944890e-02, 7.220143593892205e-02, 7.270704823541310e-02, 7.220143593892205e-02, 
7.068459904944890e-02, 6.815653756699364e-02, 6.461725149155630e-02, 6.006674082313684e-02, 5.450500556173528e-02, 4.793204570735163e-02, 
4.034786125998586e-02, 3.175245221963800e-02, 2.214581858630803e-02, 1.152796035999597e-02, -1.011224592982007e-04, -1.274142987157447e-02, 
-2.639296187683284e-02, -4.105571847507331e-02 
};

/*float fcoef15[] = { // smoothing differential FIR filter 31 taps
1.805890978092128e-01, 7.811152320165679e-02, -5.654678698256484e-03, -7.204603451250098e-02, -1.223990708630513e-01, -1.580503143718819e-01,
-1.803362916609674e-01, -1.905935293522823e-01, -1.901585540678009e-01, -1.803678924294981e-01, -1.625580710593481e-01, -1.380656165793256e-01,
-1.082270556114050e-01, -7.437891477756088e-02, -3.785772069976773e-02, -2.168078613171118e-17, 3.785772069976766e-02, 7.437891477756081e-02,
1.082270556114050e-01, 1.380656165793255e-01, 1.625580710593481e-01, 1.803678924294981e-01, 1.901585540678010e-01, 1.905935293522822e-01, 
1.803362916609674e-01, 1.580503143718819e-01, 1.223990708630513e-01, 7.204603451250102e-02, 5.654678698256546e-03, -7.811152320165669e-02, -1.805890978092130e-01 
};
*/

/****************************************************************************/
/*																			*/
/*  																		*/
/*																			*/
/****************************************************************************/



	int sockfda,sockfdb, sockfin;
	struct addrinfo hintsa,hintsb, *servinfoa, *servinfob, *porta,*portb;
	int rva,rvb;


	float x[INIT_FLTR], y[INIT_FLTR], u[INIT_FLTR], v[INIT_FLTR];
	double lpx[FRST_FLTR], lpy[FRST_FLTR], lpu[FRST_FLTR], lpv[FRST_FLTR];
	double hpx[FRST_FLTR], hpy[FRST_FLTR], hpu[FRST_FLTR], hpv[FRST_FLTR];
	
	int		i = 0, j = 0, k = 0, l = 0, m = 0, n = 0, p = 0;
	int		ii = 0, jj = 0, kk = 0, ll = 0, mm = 0, nn = 0, oo = 0, pp = 0, qq = 0;
	float kdiv = 0.0, kfft = 0.0;


	double 	Omega ;
	double 	Damping; // 0.00455 	

	double	integ_0_in, integ_1_in, integ_2_in, integ_3_in, integ_4_in, integ_5_in, integ_6_in, integ_7_in, integ_8_in;
	double	integ_0_out = 0.0, integ_1_out = 0.0, integ_2_out = 0.0, integ_3_out = 0.0, integ_4_out = 0.0, integ_5_out = 0.0, integ_6_out = 0.0, integ_7_out = MARK_freq; 
	double	integ_8_out = 0.0;
	
	double	integ_9_in, integ_10_in, integ_11_in, integ_12_in, integ_13_in, integ_14_in, integ_15_in, integ_16_in;
	double	integ_9_out = 0.0, integ_10_out = 0.0, integ_11_out = 0.0, integ_12_out = 0.0, integ_13_out = 0.0, integ_14_out = 0.0, integ_15_out = 0.0, integ_16_out = SPACE_freq; 

	double	lug_0 = 0, lug_1 = 0, lug_2 = 0, lug_3 = 0, lug_4 = 0, lug_5 = 0, lug_6 = 0, lug_7 = 0; 
	double	lug_8 = 0, lug_9 = 0, lug_10 = 0, lug_11 = 0, lug_12 = 0, lug_13 = 0, lug_14 = 0, lug_15 = 0; 

	double	COMPARATOR_0_out = -1, COMPARATOR_1_out = 0;
	
	
	int 	hystFlag = FALSE; 
	float 	resample = 0.0; // allowing for variable rates of sampling



// gaussian filter
	int 	gaussiancalcflag = FALSE;
	int 	dataIndx = 0;
	float 	gaussA = 0.0, gaussB = 0.0, gaussC = 0.0;
	float 	dataA[GAUSS_TAPS], dataB[GAUSS_TAPS], dataC[GAUSS_TAPS];
// smoothing differential filter
	int		commencesmoothingflag = FALSE;	
	int 	sgfilterIndx = 0, sgfilterOut = 0; 
	float 	sgfilter = 0.0;
	float	sgfilterdata[DIFFERENTIAL_TAPS];

//	
	int 	numbytes, hostipFlag = TRUE, hostipCount = 0;
	char 	hostip[20];
	long 	int counter = 0;
	float 	dataAavg = 0.0, dataBavg = 0.0, dataCavg = 0.0;
	int		avgsum = 0;


/****************************************************************************/
/*																			*/
/*   get sockaddr, IPv4 or IPv6:											*/
/*																			*/
/****************************************************************************/

void *get_in_addr(struct sockaddr *sa)
{
	if (sa->sa_family == AF_INET) {
		return &(((struct sockaddr_in*)sa)->sin_addr);
	}

	return &(((struct sockaddr_in6*)sa)->sin6_addr);
}

/****************************************************************************/
/*																			*/
/*  parse incoming UDP controls												*/
/*																			*/
/****************************************************************************/
void *controls_loop(void *threadarg)
{
	struct control *DataPtr;
	int sockfd;
	struct addrinfo hints, *servinfo, *p;
	int rv, opt;
	size_t i;
	int numbytes;
	struct sockaddr_storage their_addr;
	char buf[MAXBUFLEN];
	socklen_t addr_len;
	char s[INET6_ADDRSTRLEN];
	wordexp_t newargv;
	float	tempfloat = 0.0;
	
	DataPtr = (struct control *) threadarg;
	if ( DataPtr->In_flg == TRUE) {
		fprintf(stderr, "Listening on port %s\n", "9223"); fflush(stderr);
	
		memset(&hints, 0, sizeof hints);
		hints.ai_family = AF_UNSPEC; // set to AF_INET to force IPv4
		hints.ai_socktype = SOCK_DGRAM;
		hints.ai_flags = AI_PASSIVE; // use my IP


		if ((rv = getaddrinfo(NULL, "9223", &hints, &servinfo)) != 0) { // set it to 9223 for now
			fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
			exit (1);
		}

		// loop through all the results and bind to the first we can
		for(p = servinfo; p != NULL; p = p->ai_next) {
			if ((sockfd = socket(p->ai_family, p->ai_socktype,
					p->ai_protocol)) == -1) {
				perror("listener: socket");
				continue;
			}

			if (bind(sockfd, p->ai_addr, p->ai_addrlen) == -1) {
				close(sockfd);
				perror("listener: bind");
				continue;
			}

			break;
		}

		if (p == NULL) {
			fprintf(stderr, "listener: failed to bind socket\n");
			exit (2);

		}

		freeaddrinfo(servinfo);

		//	printf("listener: waiting to receive...\n");
		while (1) {
			addr_len = sizeof their_addr;
			if ((numbytes = recvfrom(sockfd, buf, MAXBUFLEN-1 , 0,
				(struct sockaddr *)&their_addr, &addr_len)) == -1) {
				perror("recvfrom");
				exit(1);
			}

		//	printf("listener: got packet from %s\n",
				inet_ntop(their_addr.ss_family, get_in_addr((struct sockaddr *)&their_addr), s, sizeof s);
		//	printf("listener: packet is %d bytes long\n", numbytes);
			buf[numbytes] = '\0';
//			fprintf(stderr, "%s\n", buf);fflush(stderr);

		//convert string with wordexp
			newargv.we_offs = 1;
			wordexp(buf, &newargv, WRDE_DOOFFS);

			for(i=1; i < newargv.we_wordc; i++) {
				if ( !strcmp(newargv.we_wordv[i], "-f") ) { 
					if (i+1 <= newargv.we_wordc) {
						sscanf(newargv.we_wordv[i + 1], "%f", &tempfloat); // frequency
						parameters.frequency = (uint32_t) tempfloat;
						parameters.frequency = parameters.frequency - freqoffset - (int) parameters.localosc;
						verbose_set_frequency(dev, parameters.frequency);
						integ_6_out = integ_15_out = 0.0;
						integ_7_out = MARK_freq;
						integ_16_out = SPACE_freq;
						i++;
					} //else toofewargs ();
				}
				else if ( !strcmp(newargv.we_wordv[i], "-o") ) { //streaming output
					if (i+1 <= newargv.we_wordc) {
						sscanf(newargv.we_wordv[i + 1], "%f", &tempfloat); // frequency
						if (tempfloat == 0.0 ) parameters.Streaming_Flg == FALSE; 
						else  parameters.Streaming_Flg == TRUE;

					} //else toofewargs ();
				}
				else if ( !strcmp(newargv.we_wordv[i], "-a") ) { //adjust the local oscillator
					if (i+1 <= newargv.we_wordc) {
						sscanf(newargv.we_wordv[i + 1], "%f", &tempfloat); // frequency
						if ((tempfloat > -400) && (tempfloat < 400)) {
							parameters.increment = parameters.increment - tempfloat/48000.0;
							if (tempfloat != 0) fprintf(stderr, "Tuning adj: %d Hz; LO => %d Hz\n", (int) tempfloat, (int) (parameters.increment*48000)); 
							if (tempfloat == 0) {
								fprintf(stderr, "\nLO: %d Hz\n", (int) (parameters.increment*48000));
								fprintf(stderr, "FSK pair: %.1f Hz : %.1f Hz : delta %.1f\n",  1000.0*integ_7_out,  1000.0*integ_16_out,  1000.0*(integ_7_out-integ_16_out)); 

								tempfloat=  (integ_7_out - MARK_freq); // 1 thousanths of Hz
								parameters.increment = parameters.increment + tempfloat/48.0; 
								//reset resonant filters
								integ_7_out = MARK_freq;
								integ_16_out = SPACE_freq;
								fprintf(stderr, "Updating: LO => %d Hz\n",  (int) (parameters.increment*48000)); 
								fprintf(stderr, "Updating FSK pair: %.1f Hz : %.1f Hz : delta %.1f\n",  1000.0*integ_7_out,  1000.0*integ_16_out,  1000.0*(integ_7_out-integ_16_out)); 
							
							}
							fflush(stderr);
						}
						else { //reset to 800 Hz
							parameters.localosc=  800;
							parameters.increment = parameters.localosc/48000.0;
							//reset resonant filters
							integ_7_out = MARK_freq;
							integ_16_out = SPACE_freq;
							fprintf(stderr, "\nLO reset: %d Hz\n", (int) (parameters.increment*48000));
					
						}
						i++;
					} //else toofewargs ();
				}
				else if ( !strcmp(newargv.we_wordv[i], "-s") ) {  //use local oscillator as test signal
					if (i+1 <= newargv.we_wordc) {
						sscanf(newargv.we_wordv[i + 1], "%f", &tempfloat); // frequency
						if (tempfloat > 0.0) {
							fprintf(stderr, "FSK upper test signal enabled\n"); fflush(stderr);
							if (parameters.localoscsv == 0.0) parameters.localoscsv = parameters.localosc; // save current value
							parameters.localosc = MARK_freq*1000; // set to MARK
							parameters.increment = parameters.localosc/48000.0; // recalculate corresponding increment
							parameters.testsignal = TRUE; // enable test
						} 
						else if (tempfloat < 0.0) {
							fprintf(stderr, "FSK lower test signal enabled\n"); fflush(stderr);
							if (parameters.localoscsv == 0.0) parameters.localoscsv = parameters.localosc; // save current value if not already saved
							parameters.localosc=  SPACE_freq*1000;
							parameters.increment = parameters.localosc/48000.0;
							parameters.testsignal = TRUE;
						}
						else {
							if (parameters.localoscsv != 0.0) {
								parameters.localosc=parameters.localoscsv; // restore previous value
								parameters.increment = parameters.localosc/48000.0;
							}
							parameters.localoscsv = 0.0; 
							parameters.testsignal = FALSE;
						}
						i++;
					} //else toofewargs ();
				}
				else if ( !strcmp(newargv.we_wordv[i], "-n") ) { //adjust the local oscillator
					if (i+1 <= newargv.we_wordc) {
						sscanf(newargv.we_wordv[i + 1], "%f", &tempfloat); // frequency
						if (tempfloat !=0) {
							parameters.signalVnoise = tempfloat;
							fprintf(stderr, "signal to noise displayed \n"); fflush(stderr);
						}
						else {
							parameters.signalVnoise = 0.0;
							fprintf(stderr, "envelope displayed \n"); fflush(stderr);
						}

						i++;
					} //else toofewargs ();
				}
				else if ( !strcmp(newargv.we_wordv[i], "-g") ) { 
					if (i+1 <= newargv.we_wordc) {
						sscanf(newargv.we_wordv[i + 1], "%f", &tempfloat); // frequency
						parameters.LNAgain = (int) (tempfloat * 10);  /* tenths of a dB */
						if (0 == parameters.LNAgain) {
							 /* Enable automatic gain */
							verbose_auto_gain(dev);
						} else {
							/* Enable manual gain */
							parameters.LNAgain = nearest_gain(dev, parameters.LNAgain);
							verbose_gain_set(dev, parameters.LNAgain);
						}
						i++;
					} //else toofewargs ();
				}
				else if ( !strcmp(newargv.we_wordv[i], "-w") ) {  //wpm
					if (i+1 <= newargv.we_wordc) {
						sscanf(newargv.we_wordv[i + 1], "%f", &tempfloat); // frequency
						parameters.resampleFlag = TRUE;
						if (tempfloat <= 0) parameters.WordsPerMinute =  25;
						else parameters.WordsPerMinute = tempfloat;
						fprintf(stderr, "WPM set to %d\n", (int) parameters.WordsPerMinute); fflush(stderr);
						i++;
					} //else toofewargs ();
				}
				else if ( !strcmp(newargv.we_wordv[i], "-r") ) {  //resonant filter
					if (i+1 <= newargv.we_wordc) {
						sscanf(newargv.we_wordv[i + 1], "%f", &tempfloat); // resonant gain
				//		if (tempfloat==0) {
				//			parameters.resonantFilterFlag = FALSE;
				//			fprintf(stderr, "Resonant filter disabled\n"); fflush(stderr);
				//		}
				//		else {
							parameters.resonantFilterFlag = TRUE; 	
							parameters.attenuate = exp(tempfloat*0.230258509)*.001; // .001 is the initial default coupling gain
							fprintf(stderr, "Resonant gain %.2f dB\n", tempfloat); fflush(stderr);
				//		}
						i++;
					} //else toofewargs ();
				}
				else if ( !strcmp(newargv.we_wordv[i], "-b") ) {  //resonant filter
					if (i+1 <= newargv.we_wordc) {
						sscanf(newargv.we_wordv[i + 1], "%f", &tempfloat); // gain
						if ((tempfloat <.5) || (tempfloat > 1.5)) {
							fprintf(stderr, "outside of range; balance set to 1.0\n"); fflush(stderr);
						}
						else {
							parameters.balance = tempfloat;
							fprintf(stderr, "balance %.2f:%.2f \n", tempfloat,1.0-tempfloat); fflush(stderr);
						}
						i++;
					} //else toofewargs ();
				}
				else if ( !strcmp(newargv.we_wordv[i], "-d") ) {  //damper
					if (i+1 <= newargv.we_wordc) {
						sscanf(newargv.we_wordv[i + 1], "%f", &tempfloat); // gain
						if ((tempfloat >= 0.5) && (tempfloat<10.0)) {
							parameters.damper = tempfloat;
							fprintf(stderr, "damper set %0.2f\n",parameters.damper); fflush(stderr);
						}
						i++;
					} //else toofewargs ();
				}
				else if ( !strcmp(newargv.we_wordv[i], "-t") ) {  //top threshold
					if (i+1 <= newargv.we_wordc) {
						sscanf(newargv.we_wordv[i + 1], "%f", &tempfloat); // threshold						
						parameters.threshold = exp((float) (tempfloat)*0.230258509);
						fprintf(stderr, "hysteresis-threshold set to %.2f dB, %f\n", tempfloat, parameters.threshold ); fflush(stderr);
						i++;
					} //else toofewargs ();
				}
				else if ( !strcmp(newargv.we_wordv[i], "-S") ) { 
					if (i+1 <= newargv.we_wordc) {
						sscanf(newargv.we_wordv[i + 1], "%f", &tempfloat); // frequency
						parameters.ScaleSpectrum = exp(tempfloat*0.230258509); // 1 is the initial default coupling gain
						fprintf(stderr, "scale spectrum set to %.2f dB, %f\n", tempfloat, parameters.ScaleSpectrum ); fflush(stderr);
						i++;
					} //else toofewargs ();
				}

			}

		}
		close(sockfd);
	
	}
   	pthread_exit((void *) 0);	

}


/****************************************************************************/
/*																			*/
/*  usage																	*/
/*																			*/
/****************************************************************************/

void usage(void)
{
	fprintf(stderr,
		"rtl_sdr, an I/Q recorder for RTL2832 based DVB-T receivers\n\n"
		"Usage:\t -f frequency_to_tune_to [Hz]\n"
		"\t[-D device_index (default: 0)]\n"
		"\t[-g gain (default: 0 for auto)]\n"
		"\t[-a gain auto]\n"
		"\t[-p ppm_error (default: 0)]\n"
		"\t[-n number of samples to read (default: 0, infinite)]\n"
		"\t[-r gain (db) (invokes resonant filter stage)]\n"
		"\t[-t noise threshold (dB) ]\n"
		"\t[-b balance FSK pair\n"
		"\t[-d [.7-10] damping resonant filter\n"
		"\t[-w WPM (resampling rate for binary output)]\n"
		"\t[-h hostname host name for displays]\n"
		"\t[-i [portno] input controls]\n"
		"\t[-S Spectral Display (db) gain]\n"
		"\tfilename (if no file name defaults to stdout)\n\n");
	exit(1);
}

/****************************************************************************/
/*																			*/
/*  signal handler												*/
/*																			*/
/****************************************************************************/

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

	
/****************************************************************************/
/*																			*/
/*  callback does I/Q filtering, downsampling, envelope, coarse graining	*/
/*																			*/
/****************************************************************************/

static void rtlsdr_callback(unsigned char *buf, uint32_t len, void *ctx)
{
	struct llist *rpt;
	
    float float_IQ[4];
	char message[80];

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
		
		while (count < (int) rpt->len) {
			i = (i + 1) % INIT_FLTR;
			j = (i + 1) % INIT_FLTR;
			x[i] = (float)rpt->data[count++]  - 127.5; //I
			u[i] = (float)rpt->data[count++]  - 127.5; //Q

			// 1274.239⋅yi = (xi + 1⋅xi-1) + 1272.239⋅yi-1	(600Hz cut off)
			y[i] = (x[i] + x[j])/163102.592 + 0.99843043573458354358*y[j];	
			v[i] = (u[i] + u[j])/163102.592 + 0.99843043573458354358*v[j];	

	// move two gaussian filter calculations forward one step each loop, to spread the computational load  ... total of 2*200 calculaions per single output.
	// these filters are operating on the data at 4800Hz ... (main loop is running at 2.4MS/s)
			if (gaussiancalcflag == TRUE){ // continue to engage in processing until all taps computed
				//fprintf(stderr, "%d: %d;  %d %f\n",gaussiancalcflag, m, l, gaussian[l]);
				gaussA = gaussA + dataA[m]*gaussian[l]; // smoothing of summed i/q without resonant filter
				gaussB = gaussB + dataB[m]*gaussian[l]; // smoothing of summed i/q with  resonant filter
				l = l + 1;
				m = (m + 1) % GAUSS_TAPS;
				if (l >= GAUSS_TAPS ) {
					l=0;
					gaussiancalcflag = FALSE; // signals finish of cycle
					commencesmoothingflag = TRUE;   // lets also trigger the smoothing calculation at this point
				} 
			}
			

			k++; 
			if (k == 50) { // downsample to 48kHz
			
				// run local oscillator 
				 t+=parameters.increment; 
				 if (t >= 1.0) t = 0.0;
				 mixer=cos(6.283185307179586477*t);	
			
			

				pp = ii;					// ii -1
				ii = (ii + 1) % FRST_FLTR;	// ii
				jj = (ii + 1) % FRST_FLTR;	// ii -6
				kk = (jj + 1) % FRST_FLTR;	// ii -5
				ll = (kk + 1) % FRST_FLTR;	// ii -4
				mm = (ll + 1) % FRST_FLTR;	// ii -3
				nn = (mm + 1) % FRST_FLTR;	// ii -2
				oo = (nn + 1) % FRST_FLTR;	// ii -2


			// mixing the LO with the incoming I/Q ( presumably already band limited to 6KHz)
			// signal processing is within the 24KHz so the 6KHz +LO is in band.

				if (parameters.testsignal == FALSE) {
					lpx[ii] = (double)  y[i]*mixer; //input to I
					lpu[ii] = (double)  v[i]*mixer; //input to Q
				} else { // test mode
					lpx[ii] = (double)  mixer/128.0; //test tone
					lpu[ii] = (double)  sin(6.283185307179586477*t)/128.0;	
				}

			// perform the LP and HP filters


				hpx[ii] = lpy[ii] = (lpx[ii] + 7*lpx[pp] + 21*lpx[oo] + 35*lpx[nn] + 35*lpx[mm] + 21*lpx[ll] + 7*lpx[kk] + lpx[jj])/denomOne
					+ consta*lpy[pp] - constb*lpy[oo] + constc*lpy[nn] - constd*lpy[mm] + conste*lpy[ll] - constf*lpy[kk] + constg*lpy[jj]; 


				hpu[ii] = lpv[ii] = (lpu[ii] + 7*lpu[pp] + 21*lpu[oo] + 35*lpu[nn] + 35*lpu[mm] + 21*lpu[ll] + 7*lpu[kk] + lpu[jj])/denomOne
					+ consta*lpv[pp] - constb*lpv[oo] + constc*lpv[nn] - constd*lpv[mm] + conste*lpv[ll] - constf*lpv[kk] + constg*lpv[jj]; 



				// 1.057⋅yi = (1⋅xi + -2⋅xi-1 + 1⋅xi-2) + 1.997⋅yi-1 + -0.946⋅yi-2
				hpy[ii] = (hpx[ii] - 2*hpx[pp] + hpx[oo])/1.057 + 1.88930936613055818353*hpy[pp] - .89498580889309366130*hpy[oo];	
				hpv[ii] = (hpu[ii] - 2*hpu[pp] + hpu[oo])/1.057 + 1.88930936613055818353*hpv[pp] - .89498580889309366130*hpv[oo];	

		// transposed from pertecs ...  auto-centered tuning with feed back from side-tuned resonant filters

				lug_0 = integ_0_out*integ_0_out + integ_1_out*integ_1_out;
				lug_1 = sqrt(integ_2_out*integ_2_out + integ_3_out*integ_3_out);
				lug_2 = integ_4_out*integ_4_out + integ_5_out*integ_5_out;
				lug_3 = integ_7_out -.01;
				lug_4 = integ_7_out;
				lug_5 = integ_7_out +.01;
				lug_6 = lug_0 - lug_2;
				lug_7 = lug_2 + lug_0;

				lug_8 = integ_9_out*integ_9_out + integ_10_out*integ_10_out;
				lug_9 = sqrt(integ_11_out*integ_11_out + integ_12_out*integ_12_out);
				lug_10 = integ_13_out*integ_13_out + integ_14_out*integ_14_out;
				lug_11 = integ_16_out -.01;
				lug_12 = integ_16_out;
				lug_13 = integ_16_out +.01;
				lug_14 = lug_8 - lug_10;
				lug_15 = lug_10 + lug_8;

		// the binary output stream determined by the realtive amplitude of the FSK pair above a threshold setting (-t)
		// don't have hysteresis 
				if (lug_1  > lug_9 + .01* parameters.threshold) COMPARATOR_0_out = 1;
				else if (lug_9  > lug_1 + .01* parameters.threshold) COMPARATOR_0_out = -1;
				else if (lug_1 < + .02* parameters.threshold) COMPARATOR_0_out = -1;


//				 if (lug_1  > integ_8_out*10 + .01*parameters.threshold*100) COMPARATOR_0_out = 1;
//				 else if (lug_9  > integ_8_out*10 + .01*parameters.threshold*100) COMPARATOR_0_out = -1;
//				 else if (lug_1 < 20*integ_8_out) COMPARATOR_0_out = -1;
		// here we set up a rlaxation timer to track reception cycle ... triggered by a dit or dah signal being received 
		// only want the auto tuning to occur durring reception cycle
				 if (COMPARATOR_0_out >= 1) COMPARATOR_1_out = 240000;
				 else if (COMPARATOR_1_out > 0) COMPARATOR_1_out--;
			   
			   
// update integrator input values
		// MARK 
				integ_0_in = integ_2_in = integ_4_in =  parameters.attenuate*hpy[ii];
				integ_1_in = integ_3_in = integ_5_in =  -parameters.attenuate*hpv[ii]; 

				integ_0_in = integ_0_in + lug_3 * ((.1310) * integ_1_out - HDAMP * integ_0_out*(1 + 1.0e-10*integ_0_out*integ_0_out ));
				integ_1_in = integ_1_in - lug_3 * ((.1310) * integ_0_out + HDAMP * integ_1_out*(1 + 1.0e-10*integ_1_out*integ_1_out ));

				integ_2_in = integ_2_in + lug_4 * ((.1310) * integ_3_out - parameters.balance*parameters.damper*HDAMP * integ_2_out*(1 + 1.0e-10*integ_2_out*integ_2_out ));		//main pair parameters.damper* 
				integ_3_in = integ_3_in - lug_4 * ((.1310) * integ_2_out + parameters.balance*parameters.damper*HDAMP * integ_3_out*(1 + 1.0e-10*integ_3_out*integ_3_out ));		//parameters.damper*


				integ_4_in = integ_4_in + lug_5 * ((.1310) * integ_5_out - HDAMP * integ_4_out*(1 + 1.0e-10*integ_4_out*integ_4_out ));
				integ_5_in = integ_5_in - lug_5 * ((.1310) * integ_4_out + HDAMP * integ_5_out*(1 + 1.0e-10*integ_5_out*integ_5_out ));
				// feedback on tuning
				integ_6_in = 1e-7*((lug_0 > lug_2)-.5)*(lug_1 > lug_9)*(COMPARATOR_1_out > 0)  - 1e-3 * integ_6_out; 
				integ_7_in = (-1.0e-3*integ_6_out   + 1e-9*(integ_7_out - MARK_freq))*(COMPARATOR_1_out > 0);
		
		// SPACE
				integ_9_in = integ_11_in = integ_13_in =  parameters.balance*parameters.attenuate*hpy[ii];
				integ_10_in = integ_12_in = integ_14_in =  -parameters.balance*parameters.attenuate*hpv[ii]; 

				integ_9_in = integ_9_in + lug_11 * ((.1310) * integ_10_out - LDAMP * integ_9_out*(1 + 1.0e-10*integ_9_out*integ_9_out ));
				integ_10_in = integ_10_in - lug_11 * ((.1310) * integ_9_out + LDAMP * integ_10_out*(1 + 1.0e-10*integ_10_out*integ_10_out ));

				integ_11_in = integ_11_in + lug_12 * ((.1310) * integ_12_out - parameters.damper*LDAMP * integ_11_out*(1 + 1.0e-10*integ_11_out*integ_11_out ));		//main pair parameters.damper* 
				integ_12_in = integ_12_in - lug_12 * ((.1310) * integ_11_out + parameters.damper*LDAMP * integ_12_out*(1 + 1.0e-10*integ_12_out*integ_12_out ));		//parameters.damper*


				integ_13_in = integ_13_in + lug_13 * ((.1310) * integ_14_out - LDAMP * integ_13_out*(1 + 1.0e-10*integ_13_out*integ_13_out ));
				integ_14_in = integ_14_in - lug_13 * ((.1310) * integ_13_out + LDAMP * integ_14_out*(1 + 1.0e-10*integ_14_out*integ_14_out ));
				// feedback on tuning
				integ_15_in = 1e-7*((lug_8 > lug_10)-.5)*(lug_9 > lug_1)*(COMPARATOR_1_out > 0)*(parameters.cwFlg == FALSE) - 1e-3 * integ_15_out; 
				integ_16_in = (-1.0e-3*integ_15_out  + 1e-9*(integ_16_out - SPACE_freq))*(COMPARATOR_1_out > 0);
		

		// update output values
				integ_0_out += integ_0_in;
				integ_1_out += integ_1_in;

				integ_2_out += integ_2_in;
				integ_3_out += integ_3_in;

				integ_4_out += integ_4_in;
				integ_5_out += integ_5_in;

				integ_6_out += integ_6_in;
				integ_7_out += integ_7_in;

//				integ_8_out += integ_8_in;

				integ_9_out += integ_9_in;
				integ_10_out += integ_10_in;

				integ_11_out += integ_11_in;
				integ_12_out += integ_12_in;

				integ_13_out += integ_13_in;
				integ_14_out += integ_14_in;

				integ_15_out += integ_15_in;
				integ_16_out += integ_16_in;



		 // end of pertecs transpose

				// lets do some averaging 
				// this computing the sum of squares of both resonant filters combined
				dataAavg += (lug_1 > lug_9)*lug_1 + (lug_9 > lug_1)*lug_9; // max 
				dataCavg += (lug_9 > lug_1)*lug_1 + (lug_1 > lug_9)*lug_9; // min

				// here the gaussian is being applied to the comparator output
				dataBavg += COMPARATOR_0_out;
				avgsum++; /// count how may have been summed
				
				// downsampling counter based on WPM rate 
				kdiv += parameters.WordsPerMinute/25; //  25 // lets make this the variable rate... at 25WPM  = 4800Hz
				kfft += 1; // 4800Hz for fft

//********************** Down to 4800Hz at 25WPM  *********************************
// In fact the downsampling is variable and tied to the WPM setting. This is a achieved by varying the time increment on kdiv 
// The rational for this is to perform gaussian filtering of the morse based on the dit/dah timing
// and then reclocking at roughly 3 (1-5) samples per dit period, i.e. 9 (6-15) samples per dah

				
				if (kdiv >= 10.0) { //drop another factor  at 25WPM its a factor of 10...  4800Hz resample for gaussian filters 


					// LOAD average of samples to INPUT GAUSSIAN FILTER
					dataA[dataIndx] = dataAavg/avgsum;
					dataB[dataIndx] = dataBavg/avgsum;
					dataC[dataIndx] = dataCavg/avgsum; // noise ratio dataA[dataIndx]/
					avgsum=0;
					dataAavg = 0.0; /// reset avg 
					dataBavg = 0.0; /// reset avg 
					dataCavg = 0.0; /// reset avg 
					
							// 200 taps in the gaussian ... working on data at 4800Hz, 
					gaussA = gaussB = gaussC = 0.0;
					m = dataIndx = (dataIndx + 1) % GAUSS_TAPS; // increment the data index, m is always being initialised to where the latest data point has been entered
					for (l=0; l < GAUSS_TAPS; l++ ) {
						gaussA = gaussA + dataA[m]*gaussian[l]; // smoothing of envelope
						gaussB = gaussB + dataB[m]*gaussian[l]; // smoothing of differential comparator output
						gaussC = gaussC + dataC[m]*gaussian[l]; // smoothing of noise assessment
						m = (m + 1) % GAUSS_TAPS;
					}							

					sgfilterdata[sgfilterIndx] = gaussC*10; 
	
					sgfilter = 0.0;
					p = sgfilterIndx;
					for (n=0; n < DIFFERENTIAL_TAPS; n++ ) {
						sgfilter = sgfilter + sgfilterdata[p]*zcoef15[n];
						p = (p + 1) % DIFFERENTIAL_TAPS;
					}
					integ_8_in = sgfilter - .001 * integ_8_out ;
					integ_8_out += integ_8_in;
					sgfilterOut = (sgfilterIndx + 15 ) % DIFFERENTIAL_TAPS;		// if want the delayed input 0 - 30 with center tap at 15	
					sgfilterIndx = (sgfilterIndx + 1) % DIFFERENTIAL_TAPS;  // increment the data register by 1 ready for next time




			
//********************** Outputting binary data stream *********************************
					if (parameters.resampleFlag==TRUE) {
						resample += 1.2; //(wpm/25.0);
						if (resample >=120){ // 25wpm if resamplestep = wpm/25.0
							snprintf(message,80, "%d",(gaussB >= 0.0)); //outputs both binary and envelope
							if ((numbytes = sendto(sockfdb, &message[0], strlen(message), 0,portb->ai_addr, portb->ai_addrlen)) == -1) {
								perror("Binary stream error");
								exit (1);
							}
							resample = 0;
						}
					}

					
				//	fprintf(stdout, "%d %g %g %g %g\n",i++, integ_2_out, integ_11_out, integ_7_out,  gaussB); fflush(stdout);

//********************** Outputting to scope via UDP on selected port *********************************
					if ((parameters.resonantFilterFlag==TRUE) && (parameters.Streaming_Flg == TRUE)) {  // outputs with resonant filter
						//differentialdata[differentialIndx] = (float)  3*logf(1+  100.0 * gaussB); 
						// outputting the I for u/l resonant filters, smoothed comparator output (CW), 
						//if (parameters.signalVnoise == 0.0) 
							snprintf(message,80, "%ld %.8f %.8f %.5f %.8f", counter , (float) integ_2_out*100 , (float) integ_11_out*100 , (float) gaussB, (float) gaussA*100 );
						// debug	fprintf(stderr, "%s\n", message);
						//else sprintf(message, "%ld %.8f %.8f %.5f %.8f",counter , (float) integ_2_out , (float) integ_11_out , (float) integ_8_out*1e-1, (float) integ_8_out ); //
									
						// send udp scope data to host
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
					}
					kdiv = 0.0; 
				}
				//********************** outputing I/Q data to fft calculation *******************************
				if (kfft >= 10.0) { //drop another factor  at 25WPM its a factor of 10...  4800Hz resample for gaussian filters 
					if (parameters.Streaming_Flg == TRUE) {
						float_IQ[0] = (float) hpy[ii]*128*parameters.ScaleSpectrum;  // I before resonant filters
						float_IQ[1] = (float) hpv[ii]*128*parameters.ScaleSpectrum;	// Q before resonant filters
						if (parameters.cwFlg == FALSE){
							float_IQ[2] = (float) (integ_2_out + integ_11_out)*parameters.ScaleSpectrum/parameters.attenuate;   //I after resonant filters
							float_IQ[3] = (float) (integ_3_out + integ_12_out)*parameters.ScaleSpectrum/parameters.attenuate;	//Q after resonant filters
						} else {
							float_IQ[2] = (float) (integ_2_out)*parameters.ScaleSpectrum/parameters.attenuate;   //I after resonant filters
							float_IQ[3] = (float) (integ_3_out)*parameters.ScaleSpectrum/parameters.attenuate;	//Q after resonant filters					
						}	
						fwrite(&float_IQ[0], sizeof(float), 4, stdout); fflush (stdout);
					}
				//*****************************************************
					kfft = 0.0; 
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

/****************************************************************************/
/*																			*/
/*  main																	*/
/*																			*/
/****************************************************************************/


int main(int argc, char **argv)
{
#ifndef _WIN32
	struct sigaction sigact;
#endif
	char *filename = NULL;
	int n_read;
	int r, opt;


	int ppm_error = 0;
	int sync_mode = 0;
	FILE *file;
	uint8_t *buffer;
	int dev_index = 0;
	int dev_given = 0;

	uint32_t samp_rate = DEFAULT_SAMPLE_RATE;
	uint32_t out_block_size = DEFAULT_BUF_LENGTH;

	parameters.In_flg = TRUE;
	parameters.attenuate = 0.01;
	parameters.balance = 1.0;
	parameters.damper = 1.0;
	parameters.threshold = 50.0;
	parameters.resampleFlag=TRUE;
//	parameters.resampleFlag = FALSE; check this out
	parameters.WordsPerMinute =  25;
	parameters.cwFlg = FALSE;

	parameters.localosc=LO; // initialised for 800Hz local oscillator
 	parameters.localoscsv=0.0;
	parameters.frequency = defaultfreq - freqoffset - (int) parameters.localosc; // tuned frequency is above ... lower sid band.
	parameters.increment=parameters.localosc/48000.0; // initialising increment
	parameters.testsignal = FALSE; // can use local oscilator as a test signal
	parameters.signalVnoise = 0.0; // 

	parameters.direct_sampling=0;
	parameters.agcFlag = FALSE;
	parameters.LNAgain = 0;

	parameters.enable_biastee = 0;
	parameters.ScaleSpectrum = 1.0;
	parameters.Streaming_Flg = TRUE; // streming of udp signal and spectral data
	
	strcpy(&hostip[0], "127.0.0.1"); /// local host is the default for the decoding. 

	while ((opt = getopt(argc, argv, "TD:f:g:a:n:p:d:r:b:w:t:h:s:c:q")) != -1) {
		switch (opt) {
		case 'D':
			dev_index = verbose_device_search(optarg);
			dev_given = 1;
			break;
		case 'f':
			parameters.frequency = (uint32_t)atofs(optarg) - freqoffset - (int) parameters.localosc; //
			break;
		case 'g':
			parameters.LNAgain = (int)(atof(optarg) * 10); /* tenths of a dB */
			break;
		case 'a': // includes the input gain for resonant filter stage
			parameters.agcFlag = TRUE;
		case 'r': // includes the input gain for resonant filter stage
			parameters.resonantFilterFlag = TRUE;						
			parameters.attenuate = exp(((float) (atof(optarg)))*0.230258509)*.001; // .001 is the initial default coupling gain
			break;
		case 'b': //  balance
			parameters.balance = atof(optarg); // default is 1.0 
			break;
		case 'c': //  CW rather than FSK
			if (atoi(optarg) == 1) parameters.cwFlg = TRUE;
			else  parameters.cwFlg = FALSE;
			break;
		case 'd': // Damping for resonant filter
			parameters.damper = (float) (atof(optarg)); // should be in the range of .5-10 say default is 1.0
			if (parameters.damper < 0.45) parameters.damper = .45; 
			break;
		case 'w': // takes a wpm argument
			parameters.resampleFlag=TRUE;
			parameters.WordsPerMinute = (float) (atof(optarg));
			break;
		case 't': //threshold top level
			parameters.threshold = exp(((float) (atof(optarg)))*0.230258509);
			break;
		case 'h': // host ip
			hostipFlag = TRUE;
			hostipCount = 0;
			strcpy(&hostip[0], optarg);
			break;
		case 'p':
			ppm_error = atoi(optarg);
			break;
		case 'n':
			bytes_to_read = (uint32_t)atof(optarg) * 2;
			break;
		case 's': // host ip
			if (atof(optarg) == 1) {
				if (parameters.localoscsv == 0.0) parameters.localoscsv = parameters.localosc;
				parameters.localosc = MARK_freq*1000;
				parameters.increment = parameters.localosc/48000.0;
				parameters.testsignal = TRUE;
			} else if (atof(optarg) == -1) {
				if (parameters.localoscsv == 0.0) parameters.localoscsv = parameters.localosc;
				parameters.localosc=  SPACE_freq*1000;
				parameters.increment = parameters.localosc/48000.0;
				parameters.testsignal = TRUE;
			} else {
				if (parameters.localoscsv != 0.0) parameters.localosc=parameters.localoscsv; // restore previous value
				parameters.localoscsv = 0.0; 
				parameters.increment = parameters.localosc/48000.0;
				parameters.testsignal = FALSE;
			}
			break;
		case 'T':
			parameters.enable_biastee = 1;
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

	//	fprintf(stderr, "Local Oscillator increment %.16f\n", parameters.increment);

/*************** start input controls-monitor thread  **********************/
	
	pthread_create(&controls_thread, NULL, controls_loop, (void *) (&parameters));

//*************** start UDP data display and binary stream outputs *********/
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


//*********************** set up rtl_sdr ********************************/

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

	if (parameters.frequency < 24000000){
		verbose_direct_sampling(dev, parameters.direct_sampling = 2);
	}	

	/* Set the frequency */

	verbose_set_frequency(dev, parameters.frequency);
	verbose_ppm_set(dev, ppm_error);

	if (0 == parameters.LNAgain) {
		 /* Enable automatic gain */
		verbose_auto_gain(dev);
	} else {
		/* Enable manual gain */
		parameters.LNAgain = nearest_gain(dev, parameters.LNAgain);
		verbose_gain_set(dev, parameters.LNAgain);
	}

	/*  AGC on */
	if (parameters.agcFlag == TRUE) {
		r = rtlsdr_set_agc_mode(dev, 1);
	}

	rtlsdr_set_bias_tee(dev, parameters.enable_biastee);
	if (parameters.enable_biastee) {
		fprintf(stderr, "activated bias-T\n"); 
		fflush(stderr);
	}

	
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

	/* Reset endpoint before we start reading from it (mandatory) */
	verbose_reset_buffer(dev);

//*********************** init I/Q butterworth filters  ********************************/

	for (i=0; i < INIT_FLTR; i++) {
		x[i] = y[i] = u[i] = v[i] = 0.0;
	}
	for (ii=0; ii < FRST_FLTR; ii++) {
		lpx[ii] = lpy[ii] = lpu[ii] = lpv[ii] = 0.0;
		hpx[ii] = hpy[ii] = hpu[ii] = hpv[ii] = 0.0;
	}

//*********************** init gaussian smoothing filter  ********************************/
//
	for (i=0; i < GAUSS_TAPS; i++) {
			dataA[i] = dataB[i] = 0.0;
	}

//*********************** init differential FIR filter  ********************************/
//
	for (i=0; i < DIFFERENTIAL_TAPS; i++) {
			sgfilterdata[i] = 0.0;
	}
	

//*********************** start async readback from rtl_sdr  ********************************/

		fprintf(stderr, "Reading samples in async mode...\n");
		r = rtlsdr_read_async(dev, rtlsdr_callback, (void *)file,
				      0, out_block_size);

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
