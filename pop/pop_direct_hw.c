
#include <math.h>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/io.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>

static volatile int keepRunning = 1;

#define REG_SIZE 0x1000

#define CODEC_BASEADDRESS 0x01C22C00
#define CCM_BASEADDRESS       0x01C20000 // CLOCK CONTROLLER
#define CCM_PLL2_AUDIO        0x008
#define CCM_AUDIO_CODEC_CLOCK 0x140

//#define CODEC_BASEADDRESS 0x01c20800

#define BUF_SIZE 0x4000

#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

/* Codec DAC register offsets and bit fields */
#define SUN4I_CODEC_DAC_DPC                     (0x00)
#define SUN4I_CODEC_DAC_DPC_EN_DA                       (31)
#define SUN4I_CODEC_DAC_DPC_DVOL                        (12)
#define SUN4I_CODEC_DAC_FIFOC                   (0x04)
#define SUN4I_CODEC_DAC_FIFOC_DAC_FS                    (29)
#define SUN4I_CODEC_DAC_FIFOC_FIR_VERSION               (28)
#define SUN4I_CODEC_DAC_FIFOC_SEND_LASAT                (26)
#define SUN4I_CODEC_DAC_FIFOC_TX_FIFO_MODE              (24)
#define SUN4I_CODEC_DAC_FIFOC_DRQ_CLR_CNT               (21)
#define SUN4I_CODEC_DAC_FIFOC_TX_TRIG_LEVEL             (8)
#define SUN4I_CODEC_DAC_FIFOC_MONO_EN                   (6)
#define SUN4I_CODEC_DAC_FIFOC_ADDA_EN                   (7)
#define SUN4I_CODEC_DAC_FIFOC_TX_SAMPLE_BITS            (5)
#define SUN4I_CODEC_DAC_FIFOC_DAC_DRQ_EN                (4)
#define SUN4I_CODEC_DAC_FIFOC_FIFO_FLUSH                (0)
#define SUN4I_CODEC_DAC_FIFOS                   (0x08)
#define SUN4I_CODEC_DAC_TXDATA                  (0x0c)
#define SUN4I_CODEC_DAC_ACTL                    (0x10)
#define SUN4I_CODEC_DAC_ACTL_DACAENR                    (31)
#define SUN4I_CODEC_DAC_ACTL_DACAENL                    (30)
#define SUN4I_CODEC_DAC_ACTL_MIXEN                      (29)
#define SUN4I_CODEC_DAC_ACTL_LDACLMIXS                  (15)
#define SUN4I_CODEC_DAC_ACTL_RDACRMIXS                  (14)
#define SUN4I_CODEC_DAC_ACTL_LDACRMIXS                  (13)
#define SUN4I_CODEC_DAC_ACTL_MICLS                  (12)
#define SUN4I_CODEC_DAC_ACTL_MICRS                  (11)
#define SUN4I_CODEC_DAC_ACTL_DACPAS                     (8)
#define SUN4I_CODEC_DAC_ACTL_MIXPAS                     (7)
#define SUN4I_CODEC_DAC_ACTL_PA_MUTE                    (6)
#define SUN4I_CODEC_DAC_ACTL_PA_VOL                     (0)
#define SUN4I_CODEC_DAC_TUNE                    (0x14)
#define SUN4I_CODEC_DAC_DEBUG                   (0x18)

/* Codec ADC register offsets and bit fields */
#define SUN4I_CODEC_ADC_FIFOC                   (0x1c)
#define SUN4I_CODEC_ADC_FIFOC_ADC_FS                    (29)
#define SUN4I_CODEC_ADC_FIFOC_EN_AD                     (28)
#define SUN4I_CODEC_ADC_FIFOC_RX_FIFO_MODE              (24)
#define SUN4I_CODEC_ADC_FIFOC_RX_TRIG_LEVEL             (8)
#define SUN4I_CODEC_ADC_FIFOC_MONO_EN                   (7)
#define SUN4I_CODEC_ADC_FIFOC_RX_SAMPLE_BITS            (6)
#define SUN4I_CODEC_ADC_FIFOC_ADC_DRQ_EN                (4)
#define SUN4I_CODEC_ADC_FIFOC_FIFO_FLUSH                (0)
#define SUN4I_CODEC_ADC_FIFOS                   (0x20)
#define SUN4I_CODEC_ADC_RXDATA                  (0x24)
#define SUN4I_CODEC_ADC_ACTL                    (0x28)
#define SUN4I_CODEC_ADC_ACTL_ADC_R_EN                   (31)
#define SUN4I_CODEC_ADC_ACTL_ADC_L_EN                   (30)
#define SUN4I_CODEC_ADC_ACTL_PREG1EN                    (29)
#define SUN4I_CODEC_ADC_ACTL_PREG2EN                    (28)
#define SUN4I_CODEC_ADC_ACTL_VMICEN                     (27)
#define SUN4I_CODEC_ADC_ACTL_PREG1G                     (25)
#define SUN4I_CODEC_ADC_ACTL_PREG2G                    (23)
#define SUN4I_CODEC_ADC_ACTL_ADCG                      (20)

#define SUN4I_CODEC_ADC_ACTL_ADCIS                      (17)
#define SUN4I_CODEC_ADC_ACTL_MIC1NEN                    (12)
#define SUN4I_CODEC_ADC_ACTL_DITHER                      (8)
#define SUN4I_CODEC_ADC_ACTL_PA_EN                      (4)
#define SUN4I_CODEC_ADC_ACTL_DDE                        (3)
#define SUN4I_CODEC_ADC_DEBUG                   (0x2c)

/* Other various ADC registers */
#define SUN4I_CODEC_DAC_TXCNT                   (0x30)
#define SUN4I_CODEC_ADC_RXCNT                   (0x34)
#define SUN4I_CODEC_AC_SYS_VERI                 (0x38)
#define SUN4I_CODEC_AC_MIC_PHONE_CAL            (0x3c)




void intHandler(int dummy) {
    keepRunning = 0;
}


uint32_t peek(volatile char *reg, int rel_address) {

	uint32_t word;
	volatile uint32_t *newaddress = (volatile uint32_t *)(reg + rel_address);
	 word = *(newaddress) ;
	 return word;
}

void poke(volatile char *reg, int rel_address, uint32_t value) {
	volatile uint32_t *newaddress = (volatile uint32_t *)(reg + rel_address);
	//printf("POKE VALOR: %x\n", value);
	*newaddress = value;
	//msync(reg, 0x1000, MS_SYNC|MS_INVALIDATE);
	//printf("POKE VALOR ASIGNADO: %x\n", *newaddress);
}

void setbit(volatile char *reg, int offset, int bit){
	uint32_t current = peek(reg, offset);
	current |= 1 << bit;
	poke(reg, offset, current);
}

void clearbit(volatile char *reg, int offset, int bit){
	uint32_t current = peek(reg, offset);
	current &= ~(1 << bit);
	poke(reg, offset, current);
}

void setbitsmask(volatile char *reg, int offset, int bit, int mask, int value){
	uint32_t current = peek(reg, offset);
	uint32_t next ;
	next = (current & ~(mask << bit)) | ((value << bit) & (mask << bit));
	poke(reg, offset, next);
}

void start_playback(volatile char *reg) {
	setbit(reg,SUN4I_CODEC_DAC_FIFOC, SUN4I_CODEC_DAC_FIFOC_FIFO_FLUSH);
	clearbit(reg,SUN4I_CODEC_DAC_FIFOC, SUN4I_CODEC_DAC_FIFOC_DAC_DRQ_EN); //(SETBIT)
	clearbit(reg,SUN4I_CODEC_DAC_FIFOC, 3 /*IRQ_EN*/); //TEST
	clearbit(reg, SUN4I_CODEC_DAC_FIFOC, 2);
	clearbit(reg, SUN4I_CODEC_DAC_FIFOC, 1);
}

void stop_playback(volatile char *reg) {
	clearbit(reg,SUN4I_CODEC_DAC_FIFOC, SUN4I_CODEC_DAC_FIFOC_DAC_DRQ_EN);
}

void start_clock(volatile char *ccm_reg) {
	setbit(ccm_reg, CCM_PLL2_AUDIO, 31);
	setbit(ccm_reg, CCM_AUDIO_CODEC_CLOCK, 31);
	setbitsmask(ccm_reg, CCM_PLL2_AUDIO, 0, 0xff, 0x15);
	setbitsmask(ccm_reg, CCM_PLL2_AUDIO, 8, 0xff, 0x56
	);
}

void stop_clock(volatile char *ccm_reg) {
	clearbit(ccm_reg, CCM_PLL2_AUDIO, 31);
	clearbit(ccm_reg, CCM_AUDIO_CODEC_CLOCK, 31);
}

void flush_tx_fifo(volatile char *reg) {
	setbit(reg,SUN4I_CODEC_DAC_FIFOC, SUN4I_CODEC_DAC_FIFOC_FIFO_FLUSH);
}

void prepare_playback(volatile char *reg) {

         /* Flush the TX FIFO */
         flush_tx_fifo(reg);
         /* Set TX FIFO Empty Trigger Level */
		 setbitsmask(reg, SUN4I_CODEC_DAC_FIFOC, SUN4I_CODEC_DAC_FIFOC_TX_TRIG_LEVEL,
					 0x3f, 0xf);

         /* Use 64 bits FIR filter */
         clearbit(reg, SUN4I_CODEC_DAC_FIFOC,SUN4I_CODEC_DAC_FIFOC_FIR_VERSION);
         /* Send zeros when we have an underrun */
		 clearbit(reg, SUN4I_CODEC_DAC_FIFOC, SUN4I_CODEC_DAC_FIFOC_SEND_LASAT);
}

void set_hw_params_playback(volatile char *reg)
{
         /* Set DAC sample rate */
         setbitsmask(reg, SUN4I_CODEC_DAC_FIFOC, SUN4I_CODEC_DAC_FIFOC_DAC_FS,
                            7 , 0x000);
         /* Set the number of channels we want to use (mono=128 levels)*/
         setbit(reg, SUN4I_CODEC_DAC_FIFOC, SUN4I_CODEC_DAC_FIFOC_MONO_EN);

         /* Set the number of sample bits to 24 bits */
         setbit(reg, SUN4I_CODEC_DAC_FIFOC, SUN4I_CODEC_DAC_FIFOC_TX_SAMPLE_BITS);
         /* Set TX FIFO mode to padding the LSBs with 0 */
         clearbit(reg, SUN4I_CODEC_DAC_FIFOC, SUN4I_CODEC_DAC_FIFOC_TX_FIFO_MODE);
}

void codec_startup(volatile char *reg)
{
         setbit(reg, SUN4I_CODEC_DAC_DPC, SUN4I_CODEC_DAC_DPC_EN_DA);
         setbitsmask(reg, SUN4I_CODEC_DAC_FIFOC,SUN4I_CODEC_DAC_FIFOC_DRQ_CLR_CNT,
                     3, 3 );

		 //setbitsmask(reg, SUN4I_CODEC_DAC_DPC, SUN4I_CODEC_DAC_DPC_DVOL, 63, 52);
}


void setvolume(volatile char *reg, int invol, int outvol ){
		 /* vol = 0-63 */
		 setbit(reg, SUN4I_CODEC_DAC_ACTL, SUN4I_CODEC_DAC_ACTL_DACAENL);
		 setbit(reg, SUN4I_CODEC_DAC_ACTL, SUN4I_CODEC_DAC_ACTL_DACAENR);
		 setbit(reg, SUN4I_CODEC_DAC_ACTL, SUN4I_CODEC_DAC_ACTL_DACPAS);
		 clearbit(reg, SUN4I_CODEC_DAC_ACTL, SUN4I_CODEC_DAC_ACTL_MIXEN);
		 clearbit(reg, SUN4I_CODEC_DAC_ACTL, SUN4I_CODEC_DAC_ACTL_MIXPAS);
		 setbit(reg, SUN4I_CODEC_DAC_ACTL, SUN4I_CODEC_DAC_ACTL_PA_MUTE);
		 setbit(reg, SUN4I_CODEC_ADC_ACTL, SUN4I_CODEC_ADC_ACTL_PA_EN);
		 clearbit(reg, SUN4I_CODEC_DAC_ACTL, SUN4I_CODEC_DAC_ACTL_MICLS);
		 clearbit(reg, SUN4I_CODEC_DAC_ACTL, SUN4I_CODEC_DAC_ACTL_MICRS);

		 clearbit(reg, SUN4I_CODEC_DAC_FIFOC, SUN4I_CODEC_DAC_FIFOC_ADDA_EN);
		 setbitsmask(reg, SUN4I_CODEC_DAC_ACTL, SUN4I_CODEC_DAC_ACTL_PA_VOL, 63, outvol);

		 setbit(reg, SUN4I_CODEC_ADC_ACTL, SUN4I_CODEC_ADC_ACTL_ADC_L_EN);
		 setbit(reg, SUN4I_CODEC_ADC_ACTL, SUN4I_CODEC_ADC_ACTL_ADC_R_EN);
		 setbit(reg, SUN4I_CODEC_ADC_ACTL, SUN4I_CODEC_ADC_ACTL_PREG1EN);
		 clearbit(reg, SUN4I_CODEC_ADC_ACTL, SUN4I_CODEC_ADC_ACTL_PREG2EN);
		 setbit(reg, SUN4I_CODEC_ADC_ACTL, SUN4I_CODEC_ADC_ACTL_VMICEN);
		 setbitsmask(reg, SUN4I_CODEC_ADC_ACTL, SUN4I_CODEC_ADC_ACTL_PREG1G, 0x3, 0);
		 setbitsmask(reg, SUN4I_CODEC_ADC_ACTL, SUN4I_CODEC_ADC_ACTL_PREG2G, 0x3, 0);
		 setbitsmask(reg, SUN4I_CODEC_ADC_ACTL, SUN4I_CODEC_ADC_ACTL_ADCG, 0x3, 3);
		 setbitsmask(reg, SUN4I_CODEC_ADC_ACTL, SUN4I_CODEC_ADC_ACTL_ADCIS, 0x7, 0b010);
	     setbit(reg, SUN4I_CODEC_ADC_ACTL, SUN4I_CODEC_ADC_ACTL_MIC1NEN);
		 clearbit(reg, SUN4I_CODEC_ADC_ACTL, SUN4I_CODEC_ADC_ACTL_DITHER);
	     clearbit(reg, SUN4I_CODEC_ADC_ACTL, SUN4I_CODEC_ADC_ACTL_DDE);

}

void setinputgain(volatile char *reg, int ingain ){
  setbitsmask(reg, SUN4I_CODEC_ADC_ACTL, SUN4I_CODEC_ADC_ACTL_PREG1G, 0x3, 0);
  setbitsmask(reg, SUN4I_CODEC_ADC_ACTL, SUN4I_CODEC_ADC_ACTL_PREG2G, 0x3, 0);
  setbitsmask(reg, SUN4I_CODEC_ADC_ACTL, SUN4I_CODEC_ADC_ACTL_ADCG, 0x3, ingain);
}
sendtodac(volatile char *reg, uint32_t sample){
	poke(reg, SUN4I_CODEC_DAC_TXDATA, sample);
}

clearirqs(volatile char *reg){
	//Clear irqs
	setbitsmask(reg, SUN4I_CODEC_DAC_FIFOS, 1, 7, 7);
}

long int time_delta_us(struct timeval before, struct timeval after){
	return (after.tv_usec + after.tv_sec * 1000000) - (before.tv_usec + before.tv_sec * 1000000);
}

void dump(volatile char *reg, volatile char *ccm_reg) {

	uint32_t i;
	printf("CLOCK REGISTER\n");
	for (i = 0; i< 0x164; i=i+4) {
		printf("%08x = %08x\n", i, peek(ccm_reg, i));
	}
	printf("CODEC REGISTER\n");
	for (i = 0; i< 0x38; i=i+4) {
		printf("%08x = %08x\n", i, peek(reg, i));
	}

}

volatile char *pop_direct_hw_init(int *fd)
{
   int ccm_fd ;
   int16_t ii = 1;
   int8_t *pi = (int8_t *) &ii;

   volatile char *reg = malloc(REG_SIZE);
   volatile char *ccm_reg = malloc(REG_SIZE);
   int pagesize = sysconf(_SC_PAGESIZE);
   int addr = CODEC_BASEADDRESS & ~(pagesize - 1);
   int offset = CODEC_BASEADDRESS & (pagesize - 1);
   int ccm_addr = CCM_BASEADDRESS & ~(pagesize - 1);
   int ccm_offset = CCM_BASEADDRESS & (pagesize - 1);



    setbuf(stdout, NULL);
    *fd = open("/dev/mem", O_RDWR);
    reg = (char *)mmap(NULL, (0x800 + pagesize - 1) & ~(pagesize - 1), PROT_WRITE | PROT_READ, MAP_SHARED, *fd, addr);
    ccm_fd = open("/dev/mem", O_RDWR);
    ccm_reg = (char *)mmap(NULL, (0x800 + pagesize - 1) & ~(pagesize - 1), PROT_WRITE | PROT_READ, MAP_SHARED, ccm_fd, ccm_addr);

	printf("ADDRESS: %x\n",addr);
	int size = (0x800 + pagesize - 1) & ~(pagesize - 1);
	printf("OFFSET: %x\n",offset);
	printf("SIZE: %x\n",size);

    if (reg == MAP_FAILED)
	{
		printf("CODEC mmap failed errno = %d\n", errno);
		exit(EXIT_FAILURE);
	}

    if (ccm_reg == MAP_FAILED)
	{
		printf("CCM mmap failed errno = %d\n", errno);
		exit(EXIT_FAILURE);
	}

    reg+=offset;
    ccm_reg+=ccm_offset;

	close(ccm_fd);
    munmap((void *)ccm_reg, 4);

    return reg;
}

void pop_direct_hw_end(char *reg, int *fd) {
    close(*fd);
    munmap((void *)reg, 4);
}

int better_sleep (double sleep_time)
{
 struct timespec tv;
 /* Construct the timespec from the number of whole seconds... */
 tv.tv_sec = (time_t) sleep_time;
 /* ... and the remainder in nanoseconds. */
 tv.tv_nsec = (long) ((sleep_time - tv.tv_sec) * 1e+9);

 while (1)
 {
  /* Sleep for the time specified in tv. If interrupted by a
    signal, place the remaining time left to sleep back into tv. */
  int rval = nanosleep (&tv, &tv);
  if (rval == 0)
   /* Completed the entire sleep time; all done. */
   return 0;
  else if (errno == EINTR)
   /* Interrupted by a signal. Try again. */
   continue;
  else
   /* Some other error; bail out. */
   return rval;
 }
 return 0;
}

void compress
    (
        float*  wav_in,     // signal
        int     n,          // N samples
        double  threshold,  // threshold (percents)
        double  slope,      // slope angle (percents)
        int     sr,         // sample rate (smp/sec)
        double  tla,        // lookahead  (ms)
        double  twnd,       // window time (ms)
        double  tatt,       // attack time  (ms)
        double  trel        // release time (ms)
    )
{
    typedef float   stereodata[2];
    stereodata*     wav = (stereodata*) wav_in; // our stereo signal
    threshold *= 0.01;          // threshold to unity (0...1)
    slope *= 0.01;              // slope to unity
    tla *= 1e-3;                // lookahead time to seconds
    twnd *= 1e-3;               // window time to seconds
    tatt *= 1e-3;               // attack time to seconds
    trel *= 1e-3;               // release time to seconds

    // attack and release "per sample decay"
    double  att = (tatt == 0.0) ? (0.0) : exp (-1.0 / (sr * tatt));
    double  rel = (trel == 0.0) ? (0.0) : exp (-1.0 / (sr * trel));

    // envelope
    double  env = 0.0;

    // sample offset to lookahead wnd start
    int     lhsmp = (int) (sr * tla);

    // samples count in lookahead window
    int     nrms = (int) (sr * twnd);

    // for each sample...
    int i = 0;
    for (i = 0; i < n; ++i)
    {
        // now compute RMS
        double  summ = 0;

        // for each sample in window
        int j = 0;
        for (j = 0; j < nrms; ++j)
        {
            int     lki = i + j + lhsmp;
            double  smp;

            // if we in bounds of signal?
            // if so, convert to mono
            if (lki < n)
                smp = 0.5 * wav[lki][0] + 0.5 * wav[lki][1];
            else
                smp = 0.0;      // if we out of bounds we just get zero in smp

            summ += smp * smp;  // square em..
        }

        double  rms = sqrt (summ / nrms);   // root-mean-square

        // dynamic selection: attack or release?
        double  theta = rms > env ? att : rel;

        // smoothing with capacitor, envelope extraction...
        // here be aware of pIV denormal numbers glitch
        env = (1.0 - theta) * rms + theta * env;

        // the very easy hard knee 1:N compressor
        double  gain = 1.0;
        if (env > threshold)
            gain = gain - (env - threshold) * slope;

        // result - two hard kneed compressed channels...
        float  leftchannel = wav[i][0] * gain;
        float  rightchannel = wav[i][1] * gain;
    }
}
