#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <math.h>
#include <sys/time.h>

#define	D_AckSte_RF_PAIRING	0
#define	D_AckSte_SB		1
#define	D_AckSte_RC		2

//#define	O_AnglDsrdMrgnLft 0.1
//#define	O_AnglDsrdMrgnRgt 0.1
//#define	O_VelTrnsDsrdMrgnLowr 0.1
//#define	O_VelTrnsDsrdMrgnUppr 0.1

#define	O_VelTrnsNmnlBck	-0.4	// ?
#define	O_VelTrnsNmnlStop	0.0	// ?
#define	O_VelTrnsNmnl1st	0.406
#define	O_VelTrnsNmnl2nd	0.6127
#define	O_VelTrnsNmnl3rd	0.81	// ?

#define	O_VelTrns1st2nd	0.51	// (0.406+0.6127)/2
#define	O_VelTrns2nd3rd	0.71	// ?(0.6127+0.8)/2

#define	O_AnglNmnlStepSpdStop	1.82
#define	O_AnglNmnlStepSpd1st	0.73
#define	O_AnglNmnlStepSpd2nd	0.76
#define	O_AnglNmnlStepSpd3rd	0.79	// ?
#define	O_AnglNmnlStepSpdBck	1.0	// ?

#define	O_VelTrnsDeadZoneLowr	0.2
#define	O_VelTrnsDeadZoneUppr	0.2
#define	O_AnglDeadZoneUppr	0.1	// ?
#define	O_AnglDeadZoneLowr	0.1	// ?

#define	O_DiamWhll (0.25+0.01)
#define	O_DiamTred (0.22)

#define	O_LPFVelTrnsNmnl	0.995
#define	O_LPFAnglNmnl		0.995

uint64_t dModeCMD_Enter = 0x6BF600030090000E;	// EnterRCModeCMD

uint64_t dCtrlCMD_LEFT  = 0x6B93000302810078;	// ControlCMD Turn LEFT
uint64_t dCtrlCMD_RIGHT = 0x6B9300030282007B;	// ControlCMD Turn RIGHT
uint64_t dCtrlCMD_UP    = 0x6B9300030283007A;	// ControlCMD Turn UP
uint64_t dCtrlCMD_DOWN  = 0x6B9300030284007D;	// ControlCMD Turn DOWN
uint64_t dCtrlCMD_STOP  = 0x6B930003028B0072;	// ControlCMD STOP
uint64_t dCtrlCMD_NULL  = 0x6B930003020000F9;	// ControlCMD NULL

typedef unsigned char byte;
typedef short int16;

typedef struct {
  byte ste_sop,ste_cmd,ste_len;byte ste_ste,ste_data1,ste_data2,ste_data3;byte ste_xor;
}
  SteMotrDrivrStrc;

typedef byte SteMotrDrivrBuf[8];

typedef union {
  SteMotrDrivrBuf buf;
  SteMotrDrivrStrc strc;
}  SteMotrDrivr;

typedef struct {
  byte spd_sop,spd_cmd,spd_len;byte spd_spdlftH,spd_spdlftL,spd_spdrgtH,spd_spdrgtL;byte spd_xor;
  byte crnt_sop,crnt_cmd,crnt_len;byte crnt_crntlftH,crnt_crntlftL,crnt_crntrgtH,crnt_crntrgtL;byte crnt_xor;
  byte attd_sop,attd_cmd,attd_len;byte attd_ptchH,attd_ptchL,attd_rollH,attd_rollL;byte attd_xor;
  byte tmpr_sop,tmpr_cmd,tmpr_len;byte tmpr_tmprlftH,tmpr_tmprlftL,tmpr_tmprrgtH,tmpr_tmprrgtL;byte tmpr_xor;
  byte dmy_sop,dmy_cmd,dmy_len;byte dmy_dmylftH,dmy_dmylftL,dmy_dmyrgtH,dmy_dmyrgtL;byte dmy_xor;
  byte stvl_sop,stvl_cmd,stvl_len;byte stvl_steH,stvl_steL,stvl_volH,stvl_volL;byte stvl_xor;
}
  InfoMotrDrivrStrc;

typedef byte InfoMotrDrivrBuf[6][8];

typedef byte InfoMotrDrivrBuf1D[6*8];

typedef union {
  InfoMotrDrivrBuf buf;
  InfoMotrDrivrStrc strc;
  InfoMotrDrivrBuf1D buf1D;
}  InfoMotrDrivr;

typedef enum {D_SpdLvelBck=-1,D_SpdLvelStop=0,D_SpdLvel1st=1,D_SpdLvel2nd=2,D_SpdLvel3rd=3} ESpdLvel;

struct termio tty_backup; /* keep pre-set */
struct termio tty_change; /* keep setting */

struct timeval gcTimePrev;

void	SetBuf8ByteRvce(uint64_t *pBufSrc, byte aBufDstn[8]);
void	PrntBuf8(byte aBuf8[8]);
void	PrntBuf6x8(byte aBuf6x8[6][8]);
int16	BindByte2Word(byte oByteH, byte oByteL);

int TrnsCtrlCMD(uint64_t* pdCtrlCMD, int cTrnsCtrlCMD, InfoMotrDrivr* uInfo, int fd);

int MotrCtrlrOpen();
void MotrCtrlrClse(int fd);

unsigned char InpKbd();

void KbdSet();
void KbdRstr();


int main()
{
    InfoMotrDrivr uInfo;

    int fd;

    unsigned char dKey;

    float oVelTrnsDsrd=0.0;
//    float oAnglDsrd=0.0;
    float oVelTrnsOdmt=0.0;
    float oAVelOdmt=0.0;

    double oAnglOdmt=0.0;

    float oVelTrnsNmnl=0.0;
    
//    int16 znVelLft,znVelRgt;

    float oTimeCycl;

    struct timeval cTimeCrnt;

    int	dSte=-1;

    int	znAVelSnsrLft,znAVelSnsrRgt;
    float	oAVelSnsrLft,oAVelSnsrRgt;

    float	oAVelDsrd=0.0;

    ESpdLvel	eSpdLvelNmnl=D_SpdLvelStop;	// bck:-1, nutr:0, 1st:1, 2nd:2, 3rd:3
    ESpdLvel	eSpdLvelDsrd=D_SpdLvelStop;	// bck:-1, nutr:0, 1st:1, 2nd:2, 3rd:3

//    float	oAnglDsrdStep=0.0;
    float	oAnglNmnlStep=0.0;
    double	oAnglNmnl=0.0;
    double	oAnglDsrd=0.0;
    
    gettimeofday(&gcTimePrev, NULL);

    fd = MotrCtrlrOpen();

    if( fd < 0 ){
      exit( -1 );
    }

    KbdSet();

    do
      {
	dKey = InpKbd();

//	puts("\033[2J");
//	puts("\x1b[2J");
	system("clear");
	
	switch(dKey)
	  {
	  case 'h':
	    puts("h");
	    oAVelDsrd += 0.1;
	    break;
	  case 'l':
	    puts("l");
	    oAVelDsrd -= 0.1;
	    break;
	  case 'k':
	    puts("k");
	    oVelTrnsDsrd += 0.1;
	    break;
	  case 'j':
	    puts("j");
	    oVelTrnsDsrd -= 0.1;
	    break;
	  case 'p':
	    puts("p");
	    oVelTrnsDsrd += 0.1;
	    break;
	  case 'n':
	    puts("n");
	    oVelTrnsDsrd -= 0.1;
	    break;
	  case 'q':
	    puts("q");
//	    oVelTrnsDsrd = 0;
//	    oAVelDsrd  = 0;
	    dSte = TrnsCtrlCMD(&dCtrlCMD_STOP,1, &uInfo, fd);
	    break;
	  default:
	    puts("NULL");
	    break;
	  }

	printf("dsrd spd: %f \t rot: %f\n",oVelTrnsDsrd,oAVelDsrd);

	// set lvel by spd nmnl 
	if(oVelTrnsNmnl < -O_VelTrnsDeadZoneLowr){
	  eSpdLvelNmnl = D_SpdLvelBck;
	  oAnglNmnlStep = O_AnglNmnlStepSpdBck;
	}
	else if((-O_VelTrnsDeadZoneLowr < oVelTrnsNmnl)&&
		(oVelTrnsNmnl <= O_VelTrnsDeadZoneUppr)){
	  eSpdLvelNmnl = D_SpdLvelStop;
	  oAnglNmnlStep = O_AnglNmnlStepSpdStop;
	}
	else if((O_VelTrnsDeadZoneUppr < oVelTrnsNmnl)&&
		(oVelTrnsNmnl <= O_VelTrns1st2nd)){
	  eSpdLvelNmnl = D_SpdLvel1st;
	  oAnglNmnlStep = O_AnglNmnlStepSpd1st;
	}
	else if((O_VelTrns1st2nd < oVelTrnsNmnl)&&
		(oVelTrnsNmnl <= O_VelTrns2nd3rd)){
	  eSpdLvelNmnl = D_SpdLvel2nd;
	  oAnglNmnlStep = O_AnglNmnlStepSpd2nd;
	}
	else if(O_VelTrns2nd3rd < oVelTrnsNmnl){
	  eSpdLvelNmnl = D_SpdLvel3rd;
	  oAnglNmnlStep = O_AnglNmnlStepSpd3rd;
	}
	else{
	  perror("no such nmnl spd lvel\n");
	}

	// set lvel by spd dsrd
	if(oVelTrnsDsrd < -O_VelTrnsDeadZoneLowr){
	  eSpdLvelDsrd = D_SpdLvelBck;
//	  oAnglDsrdStep = O_AnglNmnlStepSpdBck;
	}
	else if((-O_VelTrnsDeadZoneLowr < oVelTrnsDsrd)&&
		(oVelTrnsDsrd <= O_VelTrnsDeadZoneUppr)){
	  eSpdLvelDsrd = D_SpdLvelStop;
//	  oAnglDsrdStep = O_AnglNmnlStepSpdStop;
	}
	else if((O_VelTrnsDeadZoneUppr < oVelTrnsDsrd)&&
		(oVelTrnsDsrd <= O_VelTrns1st2nd)){
	  eSpdLvelDsrd = D_SpdLvel1st;
//	  oAnglDsrdStep = O_AnglNmnlStepSpd1st;
	}
	else if((O_VelTrns1st2nd < oVelTrnsDsrd)&&
		(oVelTrnsDsrd <= O_VelTrns2nd3rd)){
	  eSpdLvelDsrd = D_SpdLvel2nd;
//	  oAnglDsrdStep = O_AnglNmnlStepSpd2nd;
	}
	else if(O_VelTrns2nd3rd < oVelTrnsDsrd){
	  eSpdLvelDsrd = D_SpdLvel3rd;
//	  oAnglDsrdStep = O_AnglNmnlStepSpd3rd;
	}
	else{
	  perror("no such dsrd spd lvel\n");
	}

	printf("spd lvel nmnl: %d \t dsrd: %d\n",eSpdLvelNmnl,eSpdLvelDsrd);

	// lft / rgt
	// lft
	if((oAnglNmnl+O_AnglDeadZoneUppr) < oAnglDsrd){
	  oAnglNmnlStep = fabs(oAnglNmnlStep);
//	  oAnglNmnl += oAnglDsrdStep/O_LPFAnglNmnl;
	  oAnglNmnl += oAnglNmnlStep/O_LPFAnglNmnl*oTimeCycl;
//	  oAnglNmnl += oAnglNmnlStep/O_LPFAnglNmnl;
	  dSte = TrnsCtrlCMD(&dCtrlCMD_LEFT,1, &uInfo, fd);
	}
	// rgt
	else if(oAnglDsrd < (oAnglNmnl-O_AnglDeadZoneLowr)){
	  oAnglNmnlStep = -fabs(oAnglNmnlStep);
//	  oAnglNmnl -= oAnglDsrdStep/O_LPFAnglNmnl;
	  oAnglNmnl -= oAnglNmnlStep/O_LPFAnglNmnl*oTimeCycl;
//	  oAnglNmnl -= oAnglNmnlStep/O_LPFAnglNmnl;
	  dSte = TrnsCtrlCMD(&dCtrlCMD_RIGHT,1, &uInfo, fd);
	}
	// fwrd / bwrd 
	else
	  {
	    oAnglNmnlStep = 0.0;

	    if(eSpdLvelNmnl == eSpdLvelDsrd){	// skip
	      printf("spd lvel nmnl: %d \t dsrd: %d\n",eSpdLvelNmnl,eSpdLvelDsrd);
	      dSte = TrnsCtrlCMD(&dCtrlCMD_NULL,1, &uInfo, fd);
	    }
	    else{	// spd chng mtx
	      switch(eSpdLvelDsrd)
		{
		case D_SpdLvelStop:
		  switch(eSpdLvelNmnl)
		    {
		    case D_SpdLvelBck:
		      eSpdLvelNmnl = D_SpdLvelStop;
		      oVelTrnsNmnl = 0;
		      dSte = TrnsCtrlCMD(&dCtrlCMD_UP,1, &uInfo, fd);
		      break;
		    case D_SpdLvel1st:
		      eSpdLvelNmnl = D_SpdLvelStop;
		      oVelTrnsNmnl = 0;
		      dSte = TrnsCtrlCMD(&dCtrlCMD_DOWN,1, &uInfo, fd);
		      break;
		    case D_SpdLvel2nd:
		      eSpdLvelNmnl = D_SpdLvel1st;
		      oVelTrnsNmnl = O_VelTrnsNmnl1st;
		      dSte = TrnsCtrlCMD(&dCtrlCMD_DOWN,1, &uInfo, fd);
		      break;
		    case D_SpdLvel3rd:
		      eSpdLvelNmnl = D_SpdLvel2nd;
		      oVelTrnsNmnl = O_VelTrnsNmnl2nd;
		      dSte = TrnsCtrlCMD(&dCtrlCMD_DOWN,1, &uInfo, fd);
		      break;
		    default:
		      perror("in dsrd D_SpdLvelStop, no such nmnl spd lvel\n");
		      break;
		    }
		  break;
		case D_SpdLvel1st:
		  switch(eSpdLvelNmnl)
		    {
		    case D_SpdLvelBck:
		      eSpdLvelNmnl = D_SpdLvelStop;
		      oVelTrnsNmnl = 0;
		      dSte = TrnsCtrlCMD(&dCtrlCMD_UP,1, &uInfo, fd);
		      break;
		    case D_SpdLvelStop:
		      eSpdLvelNmnl = D_SpdLvel1st;
		      oVelTrnsNmnl = O_VelTrnsNmnl1st;
		      dSte = TrnsCtrlCMD(&dCtrlCMD_UP,1, &uInfo, fd);
		      break;
		    case D_SpdLvel2nd:
		      eSpdLvelNmnl = D_SpdLvel1st;
		      oVelTrnsNmnl = O_VelTrnsNmnl1st;
		      dSte = TrnsCtrlCMD(&dCtrlCMD_DOWN,1, &uInfo, fd);
		      break;
		    case D_SpdLvel3rd:
		      eSpdLvelNmnl = D_SpdLvel2nd;
		      oVelTrnsNmnl = O_VelTrnsNmnl2nd;
		      dSte = TrnsCtrlCMD(&dCtrlCMD_DOWN,1, &uInfo, fd);
		      break;
		    default:
		      perror("in dsrd D_SpdLvel1st, no such nmnl spd lvel\n");
		      break;
		    }
		  break;
		case D_SpdLvel2nd:
		  switch(eSpdLvelNmnl)
		    {
		    case D_SpdLvelBck:
		      eSpdLvelNmnl = D_SpdLvelStop;
		      oVelTrnsNmnl = 0;
		      dSte = TrnsCtrlCMD(&dCtrlCMD_UP,1, &uInfo, fd);
		      break;
		    case D_SpdLvelStop:
		      eSpdLvelNmnl = D_SpdLvel1st;
		      oVelTrnsNmnl = O_VelTrnsNmnl1st;
		      dSte = TrnsCtrlCMD(&dCtrlCMD_UP,1, &uInfo, fd);
		      break;
		    case D_SpdLvel1st:
		      eSpdLvelNmnl = D_SpdLvel2nd;
		      oVelTrnsNmnl = O_VelTrnsNmnl2nd;
		      dSte = TrnsCtrlCMD(&dCtrlCMD_UP,1, &uInfo, fd);
		      break;
		    case D_SpdLvel3rd:
		      eSpdLvelNmnl = D_SpdLvel2nd;
		      oVelTrnsNmnl = O_VelTrnsNmnl2nd;
		      dSte = TrnsCtrlCMD(&dCtrlCMD_DOWN,1, &uInfo, fd);
		      break;
		    default:
		      perror("in dsrd D_SpdLvel2nd, no such nmnl spd lvel\n");
		      break;
		    }
		  break;
		case D_SpdLvel3rd:
		  switch(eSpdLvelNmnl)
		    {
		    case D_SpdLvelBck:
		      eSpdLvelNmnl = D_SpdLvelStop;
		      oVelTrnsNmnl = 0;
		      dSte = TrnsCtrlCMD(&dCtrlCMD_UP,1, &uInfo, fd);
		      break;
		    case D_SpdLvelStop:
		      eSpdLvelNmnl = D_SpdLvel1st;
		      oVelTrnsNmnl = O_VelTrnsNmnl1st;
		      dSte = TrnsCtrlCMD(&dCtrlCMD_UP,1, &uInfo, fd);
		      break;
		    case D_SpdLvel1st:
		      eSpdLvelNmnl = D_SpdLvel2nd;
		      oVelTrnsNmnl = O_VelTrnsNmnl2nd;
		      dSte = TrnsCtrlCMD(&dCtrlCMD_UP,1, &uInfo, fd);
		      break;
		    case D_SpdLvel2nd:
		      eSpdLvelNmnl = D_SpdLvel3rd;
		      oVelTrnsNmnl = O_VelTrnsNmnl3rd;
		      dSte = TrnsCtrlCMD(&dCtrlCMD_UP,1, &uInfo, fd);
		      break;
		    default:
		      perror("in dsrd D_SpdLvel3rd, no such nmnl spd lvel\n");
		      break;
		    }
		  break;
		case D_SpdLvelBck:
		  switch(eSpdLvelNmnl)
		    {
		    case D_SpdLvelStop:
		      eSpdLvelNmnl = D_SpdLvelBck;
		      oVelTrnsNmnl = O_VelTrnsNmnlBck;
		      dSte = TrnsCtrlCMD(&dCtrlCMD_DOWN,1, &uInfo, fd);
		      break;
		    case D_SpdLvel1st:
		      eSpdLvelNmnl = D_SpdLvelStop;
		      oVelTrnsNmnl = O_VelTrnsNmnlStop;
		      dSte = TrnsCtrlCMD(&dCtrlCMD_DOWN,1, &uInfo, fd);
		      break;
		    case D_SpdLvel2nd:
		      eSpdLvelNmnl = D_SpdLvel1st;
		      oVelTrnsNmnl = O_VelTrnsNmnl1st;
		      dSte = TrnsCtrlCMD(&dCtrlCMD_DOWN,1, &uInfo, fd);
		      break;
		    case D_SpdLvel3rd:
		      eSpdLvelNmnl = D_SpdLvel2nd;
		      oVelTrnsNmnl = O_VelTrnsNmnl2nd;
		      dSte = TrnsCtrlCMD(&dCtrlCMD_DOWN,1, &uInfo, fd);
		      break;
		    default:
		      perror("in dsrd D_SpdLvelBck, no such nmnl spd lvel\n");
		      break;
		    }
		  break;
		default:
		  perror("no such dsrd spd lvel\n");
//		  dSte = TrnsCtrlCMD(&dCtrlCMD_NULL,1, &uInfo, fd);	// skip
		  break;
		}
	    }
//	    eSpdLvelPrev = eSpdLvel;
	  }
	    
	if(0==dSte){

	    znAVelSnsrLft = BindByte2Word(uInfo.strc.spd_spdlftH, uInfo.strc.spd_spdlftL);
	    znAVelSnsrRgt = BindByte2Word(uInfo.strc.spd_spdrgtH, uInfo.strc.spd_spdrgtL);
//	    PrntBuf6x8(uInfo.buf);
	    printf("spd lft: %04x \t spd rgt: %04x\n",znAVelSnsrLft,znAVelSnsrRgt);
	    oAVelSnsrLft = znAVelSnsrLft/870.0;
	    oAVelSnsrRgt = znAVelSnsrRgt/870.0;
	    printf("spd lft: %f \t spd rgt: %f\n",oAVelSnsrLft,oAVelSnsrRgt);
	    oVelTrnsOdmt = ((oAVelSnsrLft + oAVelSnsrRgt)/2)*(2*O_DiamWhll*M_PI);
	    oAVelOdmt = ((oAVelSnsrRgt - oAVelSnsrLft)/2)*(2*O_DiamWhll*M_PI)/O_DiamTred;
	    printf("crnt spd: %f \t rot: %f\n",oVelTrnsOdmt,oAVelOdmt);

	    gettimeofday(&cTimeCrnt, NULL);
	    oTimeCycl = (cTimeCrnt.tv_sec - gcTimePrev.tv_sec) + (cTimeCrnt.tv_usec - gcTimePrev.tv_usec)*1.0E-6;
	    printf("cycle time = %lf\n", oTimeCycl);
	    gcTimePrev.tv_sec  = cTimeCrnt.tv_sec;
	    gcTimePrev.tv_usec = cTimeCrnt.tv_usec;

	    oAnglDsrd += oAVelDsrd*oTimeCycl;
//	    oAnglDsrd += oAnglDsrdStep*oTimeCycl;
	    oAnglNmnl += oAnglNmnlStep*oTimeCycl;
	    printf("angl nmnl: %f \t dsrd: %f\n",oAnglNmnl,oAnglDsrd);
	    oAnglOdmt += oAVelOdmt*oTimeCycl;
	    printf("angl odmt: %f\n",oAnglOdmt);

	    oVelTrnsNmnl =
	      O_LPFVelTrnsNmnl*oVelTrnsNmnl + (1-O_LPFVelTrnsNmnl)*oVelTrnsOdmt;
//	    oAnglNmnl =
//	      O_LPFAnglNmnl*oAnglNmnl + (1-O_LPFAnglNmnl)*oAnglOdmt;
	}
	
//	usleep(50*1000);
//	usleep(30*1000);

      }while('q' != dKey);
    
    /* finish */

    KbdRstr();

    MotrCtrlrClse(fd);

    puts("bfor return.");    

    return 0;
}

void SetBuf8ByteRvce(uint64_t *pBufSrc, byte aBufDstn[8])
{
  byte	bufTmp[8];

  int	i;

  memcpy(bufTmp,pBufSrc,8);
  for(i=(1-1);i<=(8-1);i++){
    aBufDstn[i] = bufTmp[7-i];
  }
  for(i=(1-1);i<=(8-1);i++){
    printf("%02x ", aBufDstn[i]);
  }
  printf("\n");
  

  return;
}

void	PrntBuf8(byte aBuf8[8])
{
  int i;
  
  for(i=0;i<8;i++)
    {
      printf("%02x ", aBuf8[i]);
    }
  printf("\n");

  return;
}

void	PrntBuf6x8(byte aBuf6x8[6][8])
{
  int i,j;
  
  for(i=0;i<6;i++)
    {
      for(j=0;j<8;j++)
	{
	  printf("%02x ", aBuf6x8[i][j]);
	}
      printf("\n");
    }
  printf("\n");

  return;
}

int16 BindByte2Word(byte oByteH, byte oByteL)
{
  int16 roWord;

  int zoByteH,zoByteL;

  zoByteH = (oByteH & 0xFF);
  zoByteH = zoByteH << 8;
  zoByteL = (oByteL & 0xFF);
  roWord = (int16)(zoByteH | zoByteL);

  return(roWord);
}

int TrnsCtrlCMD(uint64_t* pdCtrlCMD, int cTrnsCtrlCMD, InfoMotrDrivr* uInfo, int fd)
{
  int	rdSte=-1;

  byte bufWrte[8];

  int nRead;
  int c;

//  int16 dmy01,dmy02,dmy03;
//  int16 dmy1,dmy2;
//  int16 dmy11;

  int i,j;

  InfoMotrDrivr zuInfo;

  for(c=1;c<=cTrnsCtrlCMD;c++)
    {
      SetBuf8ByteRvce(pdCtrlCMD,bufWrte);
      write( fd, bufWrte, 8 );

//      puts("bfor read.");
      nRead = read( fd, uInfo->buf, 6*8 );
      printf("nRead: %d\n",nRead);
//      PrntBuf6x8(uInfo.buf);

      if(0==nRead){
	puts("no data.");
	rdSte = -1;
	return(rdSte);
      }

      for(i=0;i<(6*8-2);i++)
	{
	  if(0x5C == uInfo->buf1D[i]){
	    if(0xE6 == uInfo->buf1D[i+1]){
	      if(0x04 == uInfo->buf1D[i+2]){
		if(i<=0){
		  puts("correct data.");
		  break;
		}
		else{
		  printf("slip data. %d \n",i);
		  //			  PrntBuf6x8(uInfo->buf);
		  for(j=0;j<(6*8-i);j++)
		    {
		      uInfo->buf1D[j] = uInfo->buf1D[j+i];
		    }
		  //			  PrntBuf6x8(uInfo.buf);
		  
		  ////			  fflush(NULL);
		  break;
		}
	      }
	    }
	  }
	}

      if((0==i)&&((6*8)==nRead)){
	rdSte = 0;
//	PrntBuf6x8(uInfo->buf);
      }
      else if((nRead-2)<=i){
	puts("no correct data.");
	fflush(NULL);
	//	   puts("bfor dmy read.");
	nRead = read( fd, uInfo->buf, 6*8 );
	printf("nRead: %d\n",nRead);
	rdSte = -1;
      }
      else{
	puts("partially correct data.");
	fflush(NULL);
	//	   puts("bfor dmy read.");
	//	   printf("nRead: %d\n",nRead);
	puts("bfor dmy read.");
	nRead = read( fd, zuInfo.buf1D, i );
	printf("nRead: %d\n",nRead);
	for(j=0;j<i;j++)
	  {
	    uInfo->buf1D[((6*8)-i)+j] = zuInfo.buf1D[j];
	  }
	rdSte = i;
//	PrntBuf6x8(uInfo->buf);
      }

//       puts("bfor sleep.");
//       usleep(50*1000);
    }	

  return(rdSte);
}

#define	N_TryOpen 10

int MotrCtrlrOpen()
{
  int riFd;

  const char dev[] = "/dev/rfcomm0";

  byte bufWrte[8];

  InfoMotrDrivr uInfo;

  int nRead;
  int	nWrte;

  int i;

  puts("bfor open.");

  riFd = open( dev, O_RDWR );
  if( riFd < 0 ){
    perror( dev );
    exit( -1 );
  }
  usleep(500*1000);
    
  for(i=0;i<N_TryOpen;i++)
    {
      SetBuf8ByteRvce(&dModeCMD_Enter,bufWrte);
      nWrte = write( riFd, bufWrte, 8 );
      printf("nWrte: %d\n",nWrte);

      usleep(100*1000);

      puts("bfor read.");
      nRead = read( riFd, uInfo.buf, 6*8 );
      printf("nRead: %d\n",nRead);
//      PrntBuf6x8(uInfo.buf);
	
      if((6*8)<=nRead)
	{
	  break;
	}

      usleep(1000*1000);
    }

  if(N_TryOpen<=i){
    perror( dev );
    exit( -1 );
  }

  usleep(100*1000);
  
  return(riFd);
}

void MotrCtrlrClse(int fd)
{
  puts("bfor close.");
  close(fd);

  return;
}

unsigned char InpKbd()
{
  unsigned char rdKey;
  
  char in_char = 0; /* hold key input */
  char read_byte = 0; /* number of input bytes */
  
  /* key input */
  in_char = '\0';
  read_byte = read(0, &in_char, 1);

  switch(read_byte)
    {
    case    -1:
      /* restore pre-set */
      ioctl(0, TCSETAF, &tty_backup);
      /* error */
      return 1;
    case    0:
      putchar('.');
      fflush(NULL);
      break;
    default:
      putchar(in_char);
      fflush(NULL);
    }

  rdKey = in_char;

  return(rdKey);
}

void KbdSet()
{
  /* store pre-set */
  ioctl(0, TCGETA, &tty_backup);
  tty_change = tty_backup;

  /* change set up values */
  tty_change.c_lflag &= ~(ECHO | ICANON); /* stop echo, change to RAW mode */
  tty_change.c_cc[VMIN] = 0;
  tty_change.c_cc[VTIME] = 1;
  ioctl(0, TCSETAF, &tty_change);
  
  return;
}

void KbdRstr()
{
  /* restore pre-set */
  ioctl(0, TCSETAF, &tty_backup);

  return;
}
