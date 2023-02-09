/* This file contains a driver for a Floppy Disk Controller (FDC) using the
 * NEC PD765 chip.  The driver supports two operations: read a block and
 * write a block.  It accepts two messages, one for reading and one for
 * writing, both using message format m2 and with the same parameters:
 *
 *    m_type      DEVICE    PROC_NR     COUNT    POSITION  ADRRESS
 * ----------------------------------------------------------------
 * |  DISK_READ | device  | proc nr |  bytes  |  offset | buf ptr |
 * |------------+---------+---------+---------+---------+---------|
 * | DISK_WRITE | device  | proc nr |  bytes  |  offset | buf ptr |
 * ----------------------------------------------------------------
 *
 * The file contains one entry point:
 *
 *   floppy_task:	main entry when system is brought up
 *
 *  Changes:
 *	27 october 1986 by Jakob Schripsema: fdc_results fixed for 8 MHz
 */

#include "../h/const.h"
#include "../h/type.h"
#include "../h/callnr.h"
#include "../h/com.h"
#include "../h/error.h"
#include "const.h"
#include "type.h"
#include "glo.h"
#include "proc.h"

/* I/O Ports used by floppy disk task. */
#define DOR            0x3F2	/* motor drive control bits */
#define FDC_STATUS     0x3F4	/* floppy disk controller status register */
#define FDC_DATA       0x3F5	/* floppy disk controller data register */
#define FDC_RATE       0x3F7	/* transfer rate register */
#define DMA_ADDR       0x004	/* port for low 16 bits of DMA address */
#define DMA_TOP        0x081	/* port for top 4 bits of 20-bit DMA addr */
#define DMA_COUNT      0x005	/* port for DMA count (count =  bytes - 1) */
#define DMA_M2         0x00C	/* DMA status port */
#define DMA_M1         0x00B	/* DMA status port */
#define DMA_INIT       0x00A	/* DMA init port */

/* Status registers returned as result of operation. */
#define ST0             0x00	/* status register 0 */
#define ST1             0x01	/* status register 1 */
#define ST2             0x02	/* status register 2 */
#define ST3             0x00	/* status register 3 (return by DRIVE_SENSE) */
#define ST_CYL          0x03	/* slot where controller reports cylinder */
#define ST_HEAD         0x04	/* slot where controller reports head */
#define ST_SEC          0x05	/* slot where controller reports sector */
#define ST_PCN          0x01	/* slot where controller reports present cyl */

/* Fields within the I/O ports. */
#define MASTER          0x80	/* used to see who is master */
#define DIRECTION       0x40	/* is FDC trying to read or write? */
#define CTL_BUSY        0x10	/* used to see when controller is busy */
#define CTL_ACCEPTING   0x80	/* bit pattern FDC gives when idle */
#define MOTOR_MASK      0xF0	/* these bits control the motors in DOR */
#define ENABLE_INT      0x0C	/* used for setting DOR port */
#define ST0_BITS        0xF8	/* check top 5 bits of seek status */
#define ST3_FAULT       0x80	/* if this bit is set, drive is sick */
#define ST3_WR_PROTECT  0x40	/* set when diskette is write protected */
#define ST3_READY       0x20	/* set when drive is ready */
#define TRANS_ST0       0x00	/* top 5 bits of ST0 for READ/WRITE */
#define SEEK_ST0        0x20	/* top 5 bits of ST0 for SEEK */
#define BAD_SECTOR      0x05	/* if these bits are set in ST1, recalibrate */
#define BAD_CYL         0x1F	/* if any of these bits are set, recalibrate */
#define WRITE_PROTECT   0x02	/* bit is set if diskette is write protected */
#define CHANGE          0xC0	/* value returned by FDC after reset */

/* Floppy disk controller command bytes. */
#define FDC_SEEK        0x0F	/* command the drive to seek */
#define FDC_READ        0xE6	/* command the drive to read */
#define FDC_WRITE       0xC5	/* command the drive to write */
#define FDC_SENSE       0x08	/* command the controller to tell its status */
#define FDC_RECALIBRATE 0x07	/* command the drive to go to cyl 0 */
#define FDC_SPECIFY     0x03	/* command the drive to accept params */

/* DMA channel commands. */
#define DMA_READ        0x46	/* DMA read opcode */
#define DMA_WRITE       0x4A	/* DMA write opcode */

/* Parameters for the disk drive. */
#define SECTOR_SIZE      512	/* physical sector size in bytes */
#define HC_SIZE         2400	/* # sectors on a high-capacity (1.2M) disk */
#define NR_HEADS        0x02	/* two heads (i.e., two tracks/cylinder) */
#define DTL             0xFF	/* determines data length (sector size) */
#define SPEC1           0xDF	/* first parameter to SPECIFY */
#define SPEC2           0x02	/* second parameter to SPECIFY */

#define MOTOR_OFF       3*HZ	/* how long to wait before stopping motor */

/* Error codes */
#define ERR_SEEK          -1	/* bad seek */
#define ERR_TRANSFER      -2	/* bad transfer */
#define ERR_STATUS        -3	/* something wrong when getting status */
#define ERR_RECALIBRATE   -4	/* recalibrate didn't work properly */
#define ERR_WR_PROTECT    -5	/* diskette is write protected */
#define ERR_DRIVE         -6	/* something wrong with a drive */

/* Miscellaneous. */
#define MOTOR_RUNNING   0xFF	/* message type for clock interrupt */
#define MAX_ERRORS        20	/* how often to try rd/wt before quitting */
#define MAX_RESULTS        8	/* max number of bytes controller returns */
#define NR_DRIVES          2	/* maximum number of drives */
#define DIVISOR          128	/* used for sector size encoding */
#define MAX_FDC_RETRY    100	/* max # times to try to output to FDC */
#define NT                 4	/* number of diskette/drive combinations */

/* Variables. */
PRIVATE struct floppy {		/* main drive struct, one entry per drive */
  int fl_opcode;		/* DISK_READ or DISK_WRITE */
  int fl_curcyl;		/* current cylinder */
  int fl_procnr;		/* which proc wanted this operation? */
  int fl_drive;			/* drive number addressed */
  int fl_cylinder;		/* cylinder number addressed */
  int fl_sector;		/* sector addressed */
  int fl_head;			/* head number addressed */
  int fl_count;			/* byte count */
  vir_bytes fl_address;		/* user virtual address */
  char fl_results[MAX_RESULTS];	/* the controller can give lots of output */
  char fl_calibration;		/* CALIBRATED or UNCALIBRATED */
  char fl_density;		/* 0 = 360K/360K; 1 = 360K/1.2M; 2= 1.2M/1.2M */
} floppy[NR_DRIVES];

#define UNCALIBRATED       0	/* drive needs to be calibrated at next use */
#define CALIBRATED         1	/* no calibration needed */

PRIVATE int motor_status;	/* current motor status is in 4 high bits */
PRIVATE int motor_goal;		/* desired motor status is in 4 high bits */
PRIVATE int prev_motor;		/* which motor was started last */
PRIVATE int need_reset;		/* set to 1 when controller must be reset */
PRIVATE int initialized;	/* set to 1 after first successful transfer */
PRIVATE int d;			/* diskette/drive combination */

PRIVATE message mess;		/* message buffer for in and out */

PRIVATE char len[] = {-1,0,1,-1,2,-1,-1,3,-1,-1,-1,-1,-1,-1,-1,4};
PRIVATE char interleave[] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};

/* Four combinations of diskette/drive are supported:
 * # Drive  diskette  Sectors  Tracks  Rotation Data-rate  Comment
 * 0  360K    360K      9       40     300 RPM  250 kbps   Standard PC DSDD
 * 1  720K    360K      9       40     300 RPM  250 kbps   Quad density PC
 * 2  1.2M    360K      9       40     360 RPM  300 kbps   PC disk in AT drive
 * 3  1.2M    1.2M     15       80     360 RPM  500 kbps   AT disk in AT drive
 */
PRIVATE int gap[NT]           = {0x2A, 0x2A, 0x23, 0x1B}; /* gap size */
PRIVATE int rate[NT]          = {0x02, 0x02, 0x01, 0x00}; /* 250,300,500 kbps*/
PRIVATE int nr_sectors[NT]    = {9,    9,    9,    15};   /* sectors/track */
PRIVATE int nr_blocks[NT]     = {720,  720,  720,  2400}; /* sectors/diskette*/
PRIVATE int steps_per_cyl[NT] = {1,    2,    2,    1};	  /* 2 = dbl step */
PRIVATE int mtr_setup[NT]     = {HZ/4,HZ/4,3*HZ/4,3*HZ/4};/* in ticks */

/*===========================================================================*
 *				floppy_task				     *
 *===========================================================================*/
PUBLIC floppy_task()
{
/* Main program of the floppy disk driver task. */

  int r, caller, proc_nr;

  /* Here is the main loop of the disk task.  It waits for a message, carries
   * it out, and sends a reply.
   */
  while (TRUE) {
	/* First wait for a request to read or write a disk block. */
	receive(ANY, &mess);	/* get a request to do some work */
	if (mess.m_source < 0)
		panic("disk task got message from ", mess.m_source);
	caller = mess.m_source;
	proc_nr = mess.PROC_NR;

	/* Now carry out the work. */
	switch(mess.m_type) {
	    case DISK_READ:	r = do_rdwt(&mess);	break;
	    case DISK_WRITE:	r = do_rdwt(&mess);	break;
	    default:		r = EINVAL;		break;
	}

	/* Finally, prepare and send the reply message. */
	mess.m_type = TASK_REPLY;
	mess.REP_PROC_NR = proc_nr;
	mess.REP_STATUS = r;	/* # of bytes transferred or error code */
	send(caller, &mess);	/* send reply to caller */
  }
}


/*===========================================================================*
 *				do_rdwt					     *

&0060:2A9D B80E00           MOV      AX,000E
&0060:2AA0 E8F517           CALL     4298
&0060:2AA3 8B5E04           MOV      BX,[BP+04]
&0060:2AA6 8B4704           MOV      AX,[BX+04]
&0060:2AA9 8946F8           MOV      [BP-08],AX
&0060:2AAC 85C0             TEST     AX,AX
&0060:2AAE 7C05             JL       2AB5
&0060:2AB0 3D0200           CMP      AX,0002
&0060:2AB3 7C06             JL       2ABB
&0060:2AB5 B8FBFF           MOV      AX,FFFB
&0060:2AB8 E9F600           JMP      2BB1
&0060:2ABB 8B46F8           MOV      AX,[BP-08]
&0060:2ABE BA1C00           MOV      DX,001C
&0060:2AC1 F7EA             IMUL     DX
&0060:2AC3 052856           ADD      AX,5628
&0060:2AC6 8BF8             MOV      DI,AX
&0060:2AC8 8B46F8           MOV      AX,[BP-08]
&0060:2ACB 894506           MOV      [DI+06],AX
&0060:2ACE 8B5E04           MOV      BX,[BP+04]
&0060:2AD1 8B4702           MOV      AX,[BX+02]
&0060:2AD4 8905             MOV      [DI],AX
&0060:2AD6 2BD2             SUB      DX,DX
&0060:2AD8 B80004           MOV      AX,0400
&0060:2ADB 52               PUSH     DX
&0060:2ADC 50               PUSH     AX
&0060:2ADD FF770C           PUSH     WORD [BX+0C]
&0060:2AE0 FF770A           PUSH     WORD [BX+0A]
&0060:2AE3 E8EC20           CALL     4BD2
&0060:2AE6 85D2             TEST     DX,DX
&0060:2AE8 7403             JZ       2AED
&0060:2AEA E9C100           JMP      2BAE
&0060:2AED 85C0             TEST     AX,AX
&0060:2AEF 7403             JZ       2AF4
&0060:2AF1 E9BA00           JMP      2BAE
&0060:2AF4 2BD2             SUB      DX,DX
&0060:2AF6 B80002           MOV      AX,0200
&0060:2AF9 52               PUSH     DX
&0060:2AFA 50               PUSH     AX
&0060:2AFB 8B5E04           MOV      BX,[BP+04]
&0060:2AFE FF770C           PUSH     WORD [BX+0C]
&0060:2B01 FF770A           PUSH     WORD [BX+0A]
&0060:2B04 E88B20           CALL     4B92
&0060:2B07 8946F2           MOV      [BP-0E],AX
&0060:2B0A 8956F4           MOV      [BP-0C],DX
&0060:2B0D 85D2             TEST     DX,DX
&0060:2B0F 7C0D             JL       2B1E
&0060:2B11 7F05             JG       2B18
&0060:2B13 3D6009           CMP      AX,0960
&0060:2B16 7206             JC       2B1E
&0060:2B18 B898FF           MOV      AX,FF98
&0060:2B1B E99300           JMP      2BB1
&0060:2B1E 8A451B           MOV      AL,[DI+1B]
&0060:2B21 98               CBW
&0060:2B22 A36A56           MOV      [566A],AX
&0060:2B25 8BD8             MOV      BX,AX
&0060:2B27 D1E3             SHL      BX,1
&0060:2B29 8B87B456         MOV      AX,[BX+56B4]
&0060:2B2D D1E0             SHL      AX,1
&0060:2B2F 99               CWD
&0060:2B30 52               PUSH     DX
&0060:2B31 50               PUSH     AX
&0060:2B32 FF76F4           PUSH     WORD [BP-0C]
&0060:2B35 FF76F2           PUSH     WORD [BP-0E]
&0060:2B38 E85720           CALL     4B92
&0060:2B3B 894508           MOV      [DI+08],AX
&0060:2B3E 8B1E6A56         MOV      BX,[566A]
&0060:2B42 D1E3             SHL      BX,1
&0060:2B44 8B87B456         MOV      AX,[BX+56B4]
&0060:2B48 99               CWD
&0060:2B49 52               PUSH     DX
&0060:2B4A 50               PUSH     AX
&0060:2B4B FF76F4           PUSH     WORD [BP-0C]
&0060:2B4E FF76F2           PUSH     WORD [BP-0E]
&0060:2B51 E87E20           CALL     4BD2
&0060:2B54 8BD8             MOV      BX,AX
&0060:2B56 8A879456         MOV      AL,[BX+5694]
&0060:2B5A 98               CBW
&0060:2B5B 89450A           MOV      [DI+0A],AX
&0060:2B5E 8B1E6A56         MOV      BX,[566A]
&0060:2B62 D1E3             SHL      BX,1
&0060:2B64 8B87B456         MOV      AX,[BX+56B4]
&0060:2B68 D1E0             SHL      AX,1
&0060:2B6A 99               CWD
&0060:2B6B 52               PUSH     DX
&0060:2B6C 50               PUSH     AX
&0060:2B6D FF76F4           PUSH     WORD [BP-0C]
&0060:2B70 FF76F2           PUSH     WORD [BP-0E]
&0060:2B73 E85C20           CALL     4BD2
&0060:2B76 8946F0           MOV      [BP-10],AX
&0060:2B79 8B1E6A56         MOV      BX,[566A]
&0060:2B7D D1E3             SHL      BX,1
&0060:2B7F 8B87B456         MOV      AX,[BX+56B4]
&0060:2B83 8946EE           MOV      [BP-12],AX
&0060:2B86 8B46F0           MOV      AX,[BP-10]
&0060:2B89 99               CWD
&0060:2B8A 8B4EEE           MOV      CX,[BP-12]
&0060:2B8D F7F9             IDIV     CX
&0060:2B8F 89450C           MOV      [DI+0C],AX
&0060:2B92 8B5E04           MOV      BX,[BP+04]
&0060:2B95 8B4708           MOV      AX,[BX+08]
&0060:2B98 89450E           MOV      [DI+0E],AX
&0060:2B9B 8B4712           MOV      AX,[BX+12]
&0060:2B9E 894510           MOV      [DI+10],AX
&0060:2BA1 8B4706           MOV      AX,[BX+06]
&0060:2BA4 894504           MOV      [DI+04],AX
&0060:2BA7 817D0E0004       CMP      [DI+0E],0400
&0060:2BAC 7406             JZ       2BB4
&0060:2BAE B8EAFF           MOV      AX,FFEA
&0060:2BB1 E91217           JMP      42C6
&0060:2BB4 C746F60000       MOV      [BP-0A],0000
&0060:2BB9 837EF614         CMP      [BP-0A],0014
&0060:2BBD 7F79             JG       2C38
&0060:2BBF FF46F6           INC      WORD [BP-0A]
&0060:2BC2 8B46F6           MOV      AX,[BP-0A]
&0060:2BC5 99               CWD
&0060:2BC6 B90500           MOV      CX,0005
&0060:2BC9 F7F9             IDIV     CX
&0060:2BCB 85D2             TEST     DX,DX
&0060:2BCD 7519             JNZ      2BE8
&0060:2BCF A16A56           MOV      AX,[566A]
&0060:2BD2 40               INC      AX
&0060:2BD3 99               CWD
&0060:2BD4 B90400           MOV      CX,0004
&0060:2BD7 F7F9             IDIV     CX
&0060:2BD9 89166A56         MOV      [566A],DX
&0060:2BDD 8BC2             MOV      AX,DX
&0060:2BDF 88451B           MOV      [DI+1B],AL
&0060:2BE2 C70666560100     MOV      [5666],0001
&0060:2BE8 8B1E6A56         MOV      BX,[566A]
&0060:2BEC D1E3             SHL      BX,1
&0060:2BEE 8B87BC56         MOV      AX,[BX+56BC]
&0060:2BF2 99               CWD
&0060:2BF3 3956F4           CMP      [BP-0C],DX
&0060:2BF6 7C07             JL       2BFF
&0060:2BF8 7FBF             JG       2BB9
&0060:2BFA 3946F2           CMP      [BP-0E],AX
&0060:2BFD 73BA             JNC      2BB9
&0060:2BFF 833E665600       CMP      [5666],0000
&0060:2C04 7403             JZ       2C09
&0060:2C06 E85206           CALL     325B
&0060:2C09 57               PUSH     DI
&0060:2C0A E86000           CALL     2C6D
&0060:2C0D 83C402           ADD      SP,0002
&0060:2C10 57               PUSH     DI
&0060:2C11 E8B401           CALL     2DC8
&0060:2C14 83C402           ADD      SP,0002
&0060:2C17 57               PUSH     DI
&0060:2C18 E86302           CALL     2E7E               ; r = seek(fp);
&0060:2C1B 83C402           ADD      SP,0002
&0060:2C1E 8946FA           MOV      [BP-06],AX
&0060:2C21 85C0             TEST     AX,AX
&0060:2C23 7594             JNZ      2BB9
&0060:2C25 57               PUSH     DI
&0060:2C26 E84403           CALL     2F6D
&0060:2C29 83C402           ADD      SP,0002
&0060:2C2C 8946FA           MOV      [BP-06],AX
&0060:2C2F 85C0             TEST     AX,AX
&0060:2C31 7405             JZ       2C38
&0060:2C33 3DFBFF           CMP      AX,FFFB
&0060:2C36 7581             JNZ      2BB9
&0060:2C38 C70662560C00     MOV      [5662],000C
&0060:2C3E B8502E           MOV      AX,2E50
&0060:2C41 50               PUSH     AX
&0060:2C42 B8B400           MOV      AX,00B4
&0060:2C45 50               PUSH     AX
&0060:2C46 E8BD06           CALL     3306
&0060:2C49 83C404           ADD      SP,0004
&0060:2C4C 837EFA00         CMP      [BP-06],0000
&0060:2C50 750C             JNZ      2C5E
&0060:2C52 837D0800         CMP      [DI+08],0000
&0060:2C56 7E06             JLE      2C5E
&0060:2C58 C70668560100     MOV      [5668],0001
&0060:2C5E 837EFA00         CMP      [BP-06],0000
&0060:2C62 7403             JZ       2C67
&0060:2C64 E94EFE           JMP      2AB5
&0060:2C67 B80004           MOV      AX,0400
&0060:2C6A E944FF           JMP      2BB1

 *===========================================================================*/
PRIVATE int do_rdwt(m_ptr)
message *m_ptr;			/* pointer to read or write message */
{
/* Carry out a read or write request from the disk. */
  register struct floppy *fp;
  int r, drive, errors, stop_motor();
  long block;

  /* Decode the message parameters. */
  drive = m_ptr->DEVICE;
  if (drive < 0 || drive >= NR_DRIVES) return(EIO);
  fp = &floppy[drive];		/* 'fp' points to entry for this drive */
  fp->fl_drive = drive;		/* save drive number explicitly */
  fp->fl_opcode = m_ptr->m_type;	/* DISK_READ or DISK_WRITE */
  if (m_ptr->POSITION % BLOCK_SIZE != 0) return(EINVAL);
  block = m_ptr->POSITION/SECTOR_SIZE;
  if (block >= HC_SIZE) return(EOF);	/* sector is beyond end of 1.2M disk */
  d = fp->fl_density;		/* diskette/drive combination */
  fp->fl_cylinder = (int) (block / (NR_HEADS * nr_sectors[d]));
  fp->fl_sector = (int) interleave[block % nr_sectors[d]];
  fp->fl_head = (int) (block % (NR_HEADS*nr_sectors[d]) )/nr_sectors[d];
  fp->fl_count = m_ptr->COUNT;
  fp->fl_address = (vir_bytes) m_ptr->ADDRESS;
  fp->fl_procnr = m_ptr->PROC_NR;
  if (fp->fl_count != BLOCK_SIZE) return(EINVAL);

  errors = 0;

  /* This loop allows a failed operation to be repeated. */
  while (errors <= MAX_ERRORS) {

	/* If a lot of errors occur when 'initialized' is 0, it probably
	 * means that we are trying at the wrong density.  Try another one.
	 * Increment 'errors' here since loop is aborted on error.
	 */
	errors++;		/* increment count once per loop cycle */
 	if (errors % (MAX_ERRORS/NT) == 0) {
 		d = (d + 1) % NT;	/* try next density */
 		fp->fl_density = d;
 		need_reset = 1;
	}
  	if (block >= nr_blocks[d]) continue;

	/* First check to see if a reset is needed. */
	if (need_reset) reset();

	/* Now set up the DMA chip. */
	dma_setup(fp);

	/* See if motor is running; if not, turn it on and wait */
	start_motor(fp);

	/* If we are going to a new cylinder, perform a seek. */
	r = seek(fp);
	if (r != OK) continue;	/* if error, try again */

	/* Perform the transfer. */
	r = transfer(fp);
	if (r == OK) break;	/* if successful, exit loop */
	if (r == ERR_WR_PROTECT) break;	/* retries won't help */

  }

  /* Start watch_dog timer to turn motor off in a few seconds */
  motor_goal = ENABLE_INT;	/* when timer goes off, kill all motors */
  clock_mess(MOTOR_OFF, stop_motor);
  if (r == OK && fp->fl_cylinder > 0) initialized = 1;	/* seek works */
  return(r == OK ? BLOCK_SIZE : EIO);
}

/*===========================================================================*
 *				dma_setup				     *
 *===========================================================================*/
PRIVATE dma_setup(fp)
struct floppy *fp;		/* pointer to the drive struct */
{
/* The IBM PC can perform DMA operations by using the DMA chip.  To use it,
 * the DMA (Direct Memory Access) chip is loaded with the 20-bit memory address
 * to be read from or written to, the byte count minus 1, and a read or write
 * opcode.  This routine sets up the DMA chip.  Note that the chip is not
 * capable of doing a DMA across a 64K boundary (e.g., you can't read a
 * 512-byte block starting at physical address 65520).
 */

  int mode, low_addr, high_addr, top_addr, low_ct, high_ct, top_end;
  vir_bytes vir, ct;
  phys_bytes user_phys;
  extern phys_bytes umap();

  mode = (fp->fl_opcode == DISK_READ ? DMA_READ : DMA_WRITE);
  vir = (vir_bytes) fp->fl_address;
  ct = (vir_bytes) fp->fl_count;
  user_phys = umap(proc_addr(fp->fl_procnr), D, vir, ct);
  low_addr  = (int) (user_phys >>  0) & BYTE;
  high_addr = (int) (user_phys >>  8) & BYTE;
  top_addr  = (int) (user_phys >> 16) & BYTE;
  low_ct  = (int) ( (ct - 1) >> 0) & BYTE;
  high_ct = (int) ( (ct - 1) >> 8) & BYTE;

  /* Check to see if the transfer will require the DMA address counter to
   * go from one 64K segment to another.  If so, do not even start it, since
   * the hardware does not carry from bit 15 to bit 16 of the DMA address.
   * Also check for bad buffer address.  These errors mean FS contains a bug.
   */
  if (user_phys == 0) panic("FS gave floppy disk driver bad addr", (int) vir);
  top_end = (int) (((user_phys + ct - 1) >> 16) & BYTE);
  if (top_end != top_addr) panic("Trying to DMA across 64K boundary", top_addr);

  /* Now set up the DMA registers. */
  lock();
  port_out(DMA_M2, mode);	/* set the DMA mode */
  port_out(DMA_M1, mode);	/* set it again */
  port_out(DMA_ADDR, low_addr);	/* output low-order 8 bits */
  port_out(DMA_ADDR, high_addr);/* output next 8 bits */
  port_out(DMA_TOP, top_addr);	/* output highest 4 bits */
  port_out(DMA_COUNT, low_ct);	/* output low 8 bits of count - 1 */
  port_out(DMA_COUNT, high_ct);	/* output high 8 bits of count - 1 */
  unlock();
  port_out(DMA_INIT, 2);	/* initialize DMA */
}


/*===========================================================================*
 *				start_motor				     *
 *===========================================================================*/
PRIVATE start_motor(fp)
struct floppy *fp;		/* pointer to the drive struct */
{
/* Control of the floppy disk motors is a big pain.  If a motor is off, you
 * have to turn it on first, which takes 1/2 second.  You can't leave it on
 * all the time, since that would wear out the diskette.  However, if you turn
 * the motor off after each operation, the system performance will be awful.
 * The compromise used here is to leave it on for a few seconds after each
 * operation.  If a new operation is started in that interval, it need not be
 * turned on again.  If no new operation is started, a timer goes off and the
 * motor is turned off.  I/O port DOR has bits to control each of 4 drives.
 * Interrupts must be disabled temporarily to prevent clock interrupt from
 * turning off motors while we are testing the bits.
 */

  int motor_bit, running, send_mess();

  lock();			/* no interrupts while checking out motor */
  motor_bit = 1 << (fp->fl_drive + 4);	/* bit mask for this drive */
  motor_goal = motor_bit | ENABLE_INT | fp->fl_drive;
  if (motor_status & prev_motor) motor_goal |= prev_motor;
  running = motor_status & motor_bit;	/* nonzero if this motor is running */
  port_out(DOR, motor_goal);
  motor_status = motor_goal;
  prev_motor = motor_bit;	/* record motor started for next time */
  unlock();

  /* If the motor was already running, we don't have to wait for it. */
  if (running) return;			/* motor was already running */
  clock_mess(mtr_setup[d], send_mess);	/* motor was not running */
  receive(CLOCK, &mess);		/* wait for clock interrupt */
}


/*===========================================================================*
 *				stop_motor				     *
 *===========================================================================*/
PRIVATE stop_motor()
{
/* This routine is called by the clock interrupt after several seconds have
 * elapsed with no floppy disk activity.  It checks to see if any drives are
 * supposed to be turned off, and if so, turns them off.
 */

  if ( (motor_goal & MOTOR_MASK) != (motor_status & MOTOR_MASK) ) {
	port_out(DOR, motor_goal);
	motor_status = motor_goal;
  }
}


/*===========================================================================*
 *				seek					     *

&0060:2E7E B80400           MOV      AX,0004
&0060:2E81 E81414           CALL     4298
&0060:2E84 8B5E04           MOV      BX,[BP+04]
&0060:2E87 807F1A00         CMP      [BX+1A],00
&0060:2E8B 7515             JNZ      2EA2
&0060:2E8D 53               PUSH     BX
&0060:2E8E E84503           CALL     31D6
&0060:2E91 83C402           ADD      SP,0002
&0060:2E94 85C0             TEST     AX,AX
&0060:2E96 740A             JZ       2EA2
&0060:2E98 B8FFFF           MOV      AX,FFFF
&0060:2E9B EB02             JMP      2E9F
&0060:2E9D 2BC0             SUB      AX,AX
&0060:2E9F E92414           JMP      42C6
&0060:2EA2 8B5E04           MOV      BX,[BP+04]
&0060:2EA5 8B4702           MOV      AX,[BX+02]
&0060:2EA8 3B4708           CMP      AX,[BX+08]
&0060:2EAB 74F0             JZ       2E9D
&0060:2EAD B80F00           MOV      AX,000F
&0060:2EB0 50               PUSH     AX
&0060:2EB1 E8D402           CALL     3188
&0060:2EB4 83C402           ADD      SP,0002
&0060:2EB7 8B5E04           MOV      BX,[BP+04]
&0060:2EBA 8B470C           MOV      AX,[BX+0C]
&0060:2EBD B90200           MOV      CX,0002
&0060:2EC0 D3E0             SHL      AX,CL
&0060:2EC2 0B4706           OR       AX,[BX+06]
&0060:2EC5 50               PUSH     AX
&0060:2EC6 E8BF02           CALL     3188
&0060:2EC9 83C402           ADD      SP,0002
&0060:2ECC 8B1E6A56         MOV      BX,[566A]
&0060:2ED0 D1E3             SHL      BX,1
&0060:2ED2 8B87C456         MOV      AX,[BX+56C4]
&0060:2ED6 8946F8           MOV      [BP-08],AX
&0060:2ED9 8B5E04           MOV      BX,[BP+04]
&0060:2EDC 8B4708           MOV      AX,[BX+08]
&0060:2EDF F76EF8           IMUL     WORD [BP-08]
&0060:2EE2 50               PUSH     AX
&0060:2EE3 E8A202           CALL     3188
&0060:2EE6 83C402           ADD      SP,0002
&0060:2EE9 833E665600       CMP      [5666],0000
&0060:2EEE 75A8             JNZ      2E98
&0060:2EF0 B86C56           MOV      AX,566C
&0060:2EF3 50               PUSH     AX
&0060:2EF4 B8FFFF           MOV      AX,FFFF
&0060:2EF7 50               PUSH     AX
&0060:2EF8 E87718           CALL     4772
&0060:2EFB 83C404           ADD      SP,0004
&0060:2EFE B80800           MOV      AX,0008
&0060:2F01 50               PUSH     AX
&0060:2F02 E88302           CALL     3188
&0060:2F05 83C402           ADD      SP,0002
&0060:2F08 FF7604           PUSH     WORD [BP+04]
&0060:2F0B E8F001           CALL     30FE
&0060:2F0E 83C402           ADD      SP,0002
&0060:2F11 8946FA           MOV      [BP-06],AX
&0060:2F14 8B5E04           MOV      BX,[BP+04]
&0060:2F17 8A4712           MOV      AL,[BX+12]
&0060:2F1A 98               CBW
&0060:2F1B 25F800           AND      AX,00F8
&0060:2F1E 3D2000           CMP      AX,0020
&0060:2F21 7405             JZ       2F28
&0060:2F23 C746FAFFFF       MOV      [BP-06],FFFF
&0060:2F28 8B1E6A56         MOV      BX,[566A]
&0060:2F2C D1E3             SHL      BX,1
&0060:2F2E 8B87C456         MOV      AX,[BX+56C4]
&0060:2F32 8946F8           MOV      [BP-08],AX
&0060:2F35 8B5E04           MOV      BX,[BP+04]
&0060:2F38 8B4708           MOV      AX,[BX+08]
&0060:2F3B F76EF8           IMUL     WORD [BP-08]
&0060:2F3E 8A5713           MOV      DL,[BX+13]
&0060:2F41 B90800           MOV      CX,0008
&0060:2F44 D3E2             SHL      DX,CL
&0060:2F46 D3FA             SAR      DX,CL
&0060:2F48 3BD0             CMP      DX,AX
&0060:2F4A 7405             JZ       2F51
&0060:2F4C C746FAFFFF       MOV      [BP-06],FFFF
&0060:2F51 837EFA00         CMP      [BP-06],0000
&0060:2F55 7410             JZ       2F67
&0060:2F57 FF7604           PUSH     WORD [BP+04]
&0060:2F5A E87902           CALL     31D6
&0060:2F5D 83C402           ADD      SP,0002
&0060:2F60 85C0             TEST     AX,AX
&0060:2F62 7403             JZ       2F67
&0060:2F64 E931FF           JMP      2E98
&0060:2F67 8B46FA           MOV      AX,[BP-06]
&0060:2F6A E932FF           JMP      2E9F

 *===========================================================================*/
PRIVATE int seek(fp)
struct floppy *fp;		/* pointer to the drive struct */
{
/* Issue a SEEK command on the indicated drive unless the arm is already
 * positioned on the correct cylinder.
 */

  int r;

  /* Are we already on the correct cylinder? */
  if (fp->fl_calibration == UNCALIBRATED)
	if (recalibrate(fp) != OK) return(ERR_SEEK);
  if (fp->fl_curcyl == fp->fl_cylinder) return(OK);

  /* No.  Wrong cylinder.  Issue a SEEK and wait for interrupt. */
  fdc_out(FDC_SEEK);		/* start issuing the SEEK command */
  fdc_out( (fp->fl_head << 2) | fp->fl_drive);
  fdc_out(fp->fl_cylinder * steps_per_cyl[d]);
  if (need_reset) return(ERR_SEEK);	/* if controller is sick, abort seek */
  receive(HARDWARE, &mess);

  /* Interrupt has been received.  Check drive status. */
  fdc_out(FDC_SENSE);		/* probe FDC to make it return status */
  r = fdc_results(fp);		/* get controller status bytes */
  if ( (fp->fl_results[ST0] & ST0_BITS) != SEEK_ST0) r = ERR_SEEK;
  if (fp->fl_results[ST1] != fp->fl_cylinder * steps_per_cyl[d]) r = ERR_SEEK;
  if (r != OK)
	if (recalibrate(fp) != OK) return(ERR_SEEK);
  return(r);
}


/*===========================================================================*
 *				transfer				     *

&0060:2F6D B80800           MOV      AX,0008
&0060:2F70 E82513           CALL     4298
&0060:2F73 8BBE0400         MOV      DI,[BP+0004]
&0060:2F77 807D1A00         CMP      [DI+1A],00
&0060:2F7B 7506             JNZ      2F83
&0060:2F7D B8FEFF           MOV      AX,FFFE
&0060:2F80 E9C700           JMP      304A
&0060:2F83 8B4506           MOV      AX,[DI+06]
&0060:2F86 050400           ADD      AX,0004
&0060:2F89 8B166056         MOV      DX,[5660]
&0060:2F8D 8BC8             MOV      CX,AX
&0060:2F8F D3FA             SAR      DX,CL
&0060:2F91 F7C20100         TEST     DX,0001
&0060:2F95 74E6             JZ       2F7D
&0060:2F97 833EC27000       CMP      [70C2],0000
&0060:2F9C 7414             JZ       2FB2
&0060:2F9E 8B1E6A56         MOV      BX,[566A]
&0060:2FA2 D1E3             SHL      BX,1
&0060:2FA4 FFB7AC56         PUSH     WORD [BX+56AC]
&0060:2FA8 B8F703           MOV      AX,03F7
&0060:2FAB 50               PUSH     AX
&0060:2FAC E85D12           CALL     420C
&0060:2FAF 83C404           ADD      SP,0004
&0060:2FB2 833D03           CMP      [DI],0003
&0060:2FB5 7505             JNZ      2FBC
&0060:2FB7 B8E600           MOV      AX,00E6
&0060:2FBA EB03             JMP      2FBF
&0060:2FBC B8C500           MOV      AX,00C5
&0060:2FBF 8946F6           MOV      [BP-0A],AX
&0060:2FC2 50               PUSH     AX
&0060:2FC3 E8C201           CALL     3188
&0060:2FC6 83C402           ADD      SP,0002
&0060:2FC9 8B450C           MOV      AX,[DI+0C]
&0060:2FCC B90200           MOV      CX,0002
&0060:2FCF D3E0             SHL      AX,CL
&0060:2FD1 0B4506           OR       AX,[DI+06]
&0060:2FD4 50               PUSH     AX
&0060:2FD5 E8B001           CALL     3188
&0060:2FD8 83C402           ADD      SP,0002
&0060:2FDB FF7508           PUSH     WORD [DI+08]
&0060:2FDE E8A701           CALL     3188
&0060:2FE1 83C402           ADD      SP,0002
&0060:2FE4 FF750C           PUSH     WORD [DI+0C]
&0060:2FE7 E89E01           CALL     3188
&0060:2FEA 83C402           ADD      SP,0002
&0060:2FED FF750A           PUSH     WORD [DI+0A]
&0060:2FF0 E89501           CALL     3188
&0060:2FF3 83C402           ADD      SP,0002
&0060:2FF6 A08856           MOV      AL,[5688]
&0060:2FF9 98               CBW
&0060:2FFA 50               PUSH     AX
&0060:2FFB E88A01           CALL     3188
&0060:2FFE 83C402           ADD      SP,0002
&0060:3001 8B1E6A56         MOV      BX,[566A]
&0060:3005 D1E3             SHL      BX,1
&0060:3007 FFB7B456         PUSH     WORD [BX+56B4]
&0060:300B E87A01           CALL     3188
&0060:300E 83C402           ADD      SP,0002
&0060:3011 8B1E6A56         MOV      BX,[566A]
&0060:3015 D1E3             SHL      BX,1
&0060:3017 FFB7A456         PUSH     WORD [BX+56A4]
&0060:301B E86A01           CALL     3188
&0060:301E 83C402           ADD      SP,0002
&0060:3021 B8FF00           MOV      AX,00FF
&0060:3024 50               PUSH     AX
&0060:3025 E86001           CALL     3188
&0060:3028 83C402           ADD      SP,0002
&0060:302B 833E665600       CMP      [5666],0000
&0060:3030 741B             JZ       304D
&0060:3032 E948FF           JMP      2F7D
&0060:3035 8B46FA           MOV      AX,[BP-06]
&0060:3038 EB10             JMP      304A
&0060:303A FF7506           PUSH     WORD [DI+06]
&0060:303D B83657           MOV      AX,5736
&0060:3040 50               PUSH     AX
&0060:3041 E84A17           CALL     478E
&0060:3044 83C404           ADD      SP,0004
&0060:3047 B8FBFF           MOV      AX,FFFB
&0060:304A E97912           JMP      42C6
&0060:304D B86C56           MOV      AX,566C
&0060:3050 50               PUSH     AX
&0060:3051 B8FFFF           MOV      AX,FFFF
&0060:3054 50               PUSH     AX
&0060:3055 E81A17           CALL     4772
&0060:3058 83C404           ADD      SP,0004
&0060:305B 57               PUSH     DI
&0060:305C E89F00           CALL     30FE
&0060:305F 83C402           ADD      SP,0002
&0060:3062 8946FA           MOV      [BP-06],AX
&0060:3065 85C0             TEST     AX,AX
&0060:3067 75CC             JNZ      3035
&0060:3069 8A4513           MOV      AL,[DI+13]
&0060:306C 98               CBW
&0060:306D A90500           TEST     AX,0005
&0060:3070 7509             JNZ      307B
&0060:3072 8A4514           MOV      AL,[DI+14]
&0060:3075 98               CBW
&0060:3076 A91F00           TEST     AX,001F
&0060:3079 7404             JZ       307F
&0060:307B C6451A00         MOV      [DI+1A],00
&0060:307F 8A4513           MOV      AL,[DI+13]
&0060:3082 98               CBW
&0060:3083 A90200           TEST     AX,0002
&0060:3086 75B2             JNZ      303A
&0060:3088 8A4512           MOV      AL,[DI+12]
&0060:308B 98               CBW
&0060:308C A9F800           TEST     AX,00F8
&0060:308F 7403             JZ       3094
&0060:3091 E9E9FE           JMP      2F7D
&0060:3094 8A4513           MOV      AL,[DI+13]
&0060:3097 98               CBW
&0060:3098 8A5514           MOV      DL,[DI+14]
&0060:309B B90800           MOV      CX,0008
&0060:309E D3E2             SHL      DX,CL
&0060:30A0 D3FA             SAR      DX,CL
&0060:30A2 0BC2             OR       AX,DX
&0060:30A4 7403             JZ       30A9
&0060:30A6 E9D4FE           JMP      2F7D
&0060:30A9 8B1E6A56         MOV      BX,[566A]
&0060:30AD D1E3             SHL      BX,1
&0060:30AF 8B87B456         MOV      AX,[BX+56B4]
&0060:30B3 8946F4           MOV      [BP-0C],AX
&0060:30B6 8A4515           MOV      AL,[DI+15]
&0060:30B9 98               CBW
&0060:30BA 2B4508           SUB      AX,[DI+08]
&0060:30BD D1E0             SHL      AX,1
&0060:30BF F76EF4           IMUL     WORD [BP-0C]
&0060:30C2 8946F8           MOV      [BP-08],AX
&0060:30C5 8B1E6A56         MOV      BX,[566A]
&0060:30C9 D1E3             SHL      BX,1
&0060:30CB 8B87B456         MOV      AX,[BX+56B4]
&0060:30CF 8946F4           MOV      [BP-0C],AX
&0060:30D2 8A4516           MOV      AL,[DI+16]
&0060:30D5 98               CBW
&0060:30D6 2B450C           SUB      AX,[DI+0C]
&0060:30D9 F76EF4           IMUL     WORD [BP-0C]
&0060:30DC 0146F8           ADD      [BP-08],AX
&0060:30DF 8A4517           MOV      AL,[DI+17]
&0060:30E2 98               CBW
&0060:30E3 2B450A           SUB      AX,[DI+0A]
&0060:30E6 0146F8           ADD      [BP-08],AX
&0060:30E9 8B46F8           MOV      AX,[BP-08]
&0060:30EC B90900           MOV      CX,0009
&0060:30EF D3E0             SHL      AX,CL
&0060:30F1 3B450E           CMP      AX,[DI+0E]
&0060:30F4 7403             JZ       30F9
&0060:30F6 E984FE           JMP      2F7D
&0060:30F9 2BC0             SUB      AX,AX
&0060:30FB E94CFF           JMP      304A

 *===========================================================================*/
PRIVATE int transfer(fp)
register struct floppy *fp;	/* pointer to the drive struct */
{
/* The drive is now on the proper cylinder.  Read or write 1 block. */

  int r, s, op;
  extern int olivetti;

  /* Never attempt a transfer if the drive is uncalibrated or motor is off. */
  if (fp->fl_calibration == UNCALIBRATED) return(ERR_TRANSFER);
  if ( ( (motor_status>>(fp->fl_drive+4)) & 1) == 0) return(ERR_TRANSFER);

  /* The PC-AT requires the date rate to be set to 250 or 500 kbps */
  if (pc_at) port_out(FDC_RATE, rate[d]);

  /* The command is issued by outputing 9 bytes to the controller chip. */
  op = (fp->fl_opcode == DISK_READ ? FDC_READ : FDC_WRITE);
  fdc_out(op);			/* issue the read or write command */
  fdc_out( (fp->fl_head << 2) | fp->fl_drive);
  fdc_out(fp->fl_cylinder);	/* tell controller which cylinder */
  fdc_out(fp->fl_head);		/* tell controller which head */
  fdc_out(fp->fl_sector);	/* tell controller which sector */
  fdc_out( (int) len[SECTOR_SIZE/DIVISOR]);	/* sector size */
  fdc_out(nr_sectors[d]);	/* tell controller how big a track is */
  fdc_out(gap[d]);		/* tell controller how big sector gap is */
  fdc_out(DTL);			/* tell controller about data length */

  /* Block, waiting for disk interrupt. */
  if (need_reset) return(ERR_TRANSFER);	/* if controller is sick, abort op */
  receive(HARDWARE, &mess);

  /* Get controller status and check for errors. */
  r = fdc_results(fp);
  if (r != OK) return(r);
  if ( (fp->fl_results[ST1] & BAD_SECTOR) || (fp->fl_results[ST2] & BAD_CYL) )
	fp->fl_calibration = UNCALIBRATED;
  if (fp->fl_results[ST1] & WRITE_PROTECT) {
	printf("Diskette in drive %d is write protected.\n", fp->fl_drive);
	return(ERR_WR_PROTECT);
  }
  if ((fp->fl_results[ST0] & ST0_BITS) != TRANS_ST0) return(ERR_TRANSFER);
  if (fp->fl_results[ST1] | fp->fl_results[ST2]) return(ERR_TRANSFER);

  /* Compare actual numbers of sectors transferred with expected number. */
  s =  (fp->fl_results[ST_CYL] - fp->fl_cylinder) * NR_HEADS * nr_sectors[d];
  s += (fp->fl_results[ST_HEAD] - fp->fl_head) * nr_sectors[d];
  s += (fp->fl_results[ST_SEC] - fp->fl_sector);
  if (s * SECTOR_SIZE != fp->fl_count) return(ERR_TRANSFER);
  return(OK);
}


/*===========================================================================*
 *				fdc_results				     *

&0060:30FE B80800           MOV      AX,0008
&0060:3101 E89411           CALL     4298               ; goto enter_proc
&0060:3104 8BBE0400         MOV      DI,[BP+0004]
&0060:3108 C746FA0000       MOV      [BP-06],0000
&0060:310D EB67             JMP      3176
&0060:310F C746F40000       MOV      [BP-0C],0000
&0060:3114 C746F80000       MOV      [BP-08],0000
&0060:3119 EB07             JMP      3122
&0060:311B 2BC0             SUB      AX,AX
&0060:311D EB66             JMP      3185
&0060:311F FF46F8           INC      WORD [BP-08]
&0060:3122 837EF864         CMP      [BP-08],0064
&0060:3126 7D1A             JGE      3142
&0060:3128 8D46F6           LEA      AX,[BP-0A]
&0060:312B 50               PUSH     AX
&0060:312C B8F403           MOV      AX,03F4
&0060:312F 50               PUSH     AX
&0060:3130 E8EB10           CALL     421E               ; port_in(FDC_STATUS, &status);
&0060:3133 83C404           ADD      SP,0004
&0060:3136 F746F68000       TEST     [BP-0A],0080
&0060:313B 74E2             JZ       311F
&0060:313D C746F40100       MOV      [BP-0C],0001
&0060:3142 837EF400         CMP      [BP-0C],0000
&0060:3146 743A             JZ       3182
&0060:3148 F746F61000       TEST     [BP-0A],0010
&0060:314D 74CC             JZ       311B
&0060:314F F746F64000       TEST     [BP-0A],0040
&0060:3154 742C             JZ       3182
&0060:3156 8D46F6           LEA      AX,[BP-0A]
&0060:3159 50               PUSH     AX
&0060:315A B8F503           MOV      AX,03F5
&0060:315D 50               PUSH     AX
&0060:315E E8BD10           CALL     421E               ; port_in(FDC_DATA, &status);

&0060:3161 83C404           ADD      SP,0004
&0060:3164 8B46F6           MOV      AX,[BP-0A]
&0060:3167 2AE4             SUB      AH,AH
&0060:3169 8D5512           LEA      DX,[DI+12]
&0060:316C 0356FA           ADD      DX,[BP-06]
&0060:316F 8BDA             MOV      BX,DX
&0060:3171 8807             MOV      [BX],AL
&0060:3173 FF46FA           INC      WORD [BP-06]
&0060:3176 837EFA08         CMP      [BP-06],0008
&0060:317A 7C93             JL       310F
&0060:317C C70666560100     MOV      [5666],0001
&0060:3182 B8FDFF           MOV      AX,FFFD
&0060:3185 E93E11           JMP      42C6               ; goto leave_proc

&0060:4298 5B               POP      BX                 ; enter_proc
&0060:4299 55               PUSH     BP
&0060:429A 8BEC             MOV      BP,SP
&0060:429C 57               PUSH     DI
&0060:429D 56               PUSH     SI
&0060:429E 2BE0             SUB      SP,AX
&0060:42A0 3B26E85D         CMP      SP,[5DE8]
&0060:42A4 7602             JBE      42A8
&0060:42A6 FFE3             JMP      BX

&0060:42C6 8D66FC           LEA      SP,[BP-04]         ; leave_proc
&0060:42C9 5E               POP      SI
&0060:42CA 5F               POP      DI
&0060:42CB 5D               POP      BP
&0060:42CC C3               RET

 *===========================================================================*/
PRIVATE int fdc_results(fp)
register struct floppy *fp;	/* pointer to the drive struct */
{
/* Extract results from the controller after an operation. */

  int i, j, status, ready;

  /* Loop, extracting bytes from FDC until it says it has no more. */
  for (i = 0; i < MAX_RESULTS; i++) {
	ready = FALSE;
	for (j = 0; j < MAX_FDC_RETRY; j++) {
		port_in(FDC_STATUS, &status);
		if (status & MASTER) {
			ready = TRUE;
			break;
		}
	}
	if (ready == FALSE) return(ERR_STATUS);	/* time out */

	if ((status & CTL_BUSY) == 0) return(OK);
	if ((status & DIRECTION) == 0) return(ERR_STATUS);
	port_in(FDC_DATA, &status);
	fp->fl_results[i] = status & BYTE;
  }

  /* FDC is giving back too many results. */
  need_reset = TRUE;		/* controller chip must be reset */
  return(ERR_STATUS);
}


/*===========================================================================*
 *				fdc_out					     *
 *===========================================================================*/
PRIVATE fdc_out(val)
int val;			/* write this byte to floppy disk controller */
{
/* Output a byte to the controller.  This is not entirely trivial, since you
 * can only write to it when it is listening, and it decides when to listen.
 * If the controller refuses to listen, the FDC chip is given a hard reset.
 */

  int retries, r;

  if (need_reset) return;	/* if controller is not listening, return */
  retries = MAX_FDC_RETRY;

  /* It may take several tries to get the FDC to accept a command. */
  while (retries-- > 0) {
	port_in(FDC_STATUS, &r);
	r &= (MASTER | DIRECTION);	/* just look at bits 2 and 3 */
	if (r != CTL_ACCEPTING) continue;	/* FDC is not listening */
	port_out(FDC_DATA, val);
	return;
  }

  /* Controller is not listening.  Hit it over the head with a hammer. */
  need_reset = TRUE;
}


/*===========================================================================*
 *				recalibrate				     *

&0060:31D6 B80200           MOV      AX,0002
&0060:31D9 E8BC10           CALL     4298               ; goto enter_proc
&0060:31DC 8BBE0400         MOV      DI,[BP+0004]
&0060:31E0 57               PUSH     DI
&0060:31E1 E8E4FB           CALL     2DC8
&0060:31E4 83C402           ADD      SP,0002
&0060:31E7 B80700           MOV      AX,0007
&0060:31EA 50               PUSH     AX
&0060:31EB E89AFF           CALL     3188
&0060:31EE 83C402           ADD      SP,0002
&0060:31F1 FF7506           PUSH     WORD [DI+06]
&0060:31F4 E891FF           CALL     3188
&0060:31F7 83C402           ADD      SP,0002
&0060:31FA 833E665600       CMP      [5666],0000
&0060:31FF 7415             JZ       3216
&0060:3201 B8FFFF           MOV      AX,FFFF
&0060:3204 EB0D             JMP      3213
&0060:3206 C70666560100     MOV      [5666],0001
&0060:320C C6451A00         MOV      [DI+1A],00
&0060:3210 B8FCFF           MOV      AX,FFFC
&0060:3213 E9B010           JMP      42C6               ; goto leave_proc
&0060:3216 B86C56           MOV      AX,566C
&0060:3219 50               PUSH     AX
&0060:321A B8FFFF           MOV      AX,FFFF
&0060:321D 50               PUSH     AX
&0060:321E E85115           CALL     4772
&0060:3221 83C404           ADD      SP,0004
&0060:3224 B80800           MOV      AX,0008
&0060:3227 50               PUSH     AX
&0060:3228 E85DFF           CALL     3188
&0060:322B 83C402           ADD      SP,0002
&0060:322E 57               PUSH     DI
&0060:322F E8CCFE           CALL     30FE
&0060:3232 83C402           ADD      SP,0002
&0060:3235 8946FA           MOV      [BP-06],AX
&0060:3238 C74502FFFF       MOV      [DI+02],FFFF
&0060:323D 85C0             TEST     AX,AX
&0060:323F 75C5             JNZ      3206
&0060:3241 8A4512           MOV      AL,[DI+12]
&0060:3244 98               CBW
&0060:3245 25F800           AND      AX,00F8
&0060:3248 3D2000           CMP      AX,0020
&0060:324B 75B9             JNZ      3206
&0060:324D 807D1300         CMP      [DI+13],00
&0060:3251 75B3             JNZ      3206
&0060:3253 C6451A01         MOV      [DI+1A],01
&0060:3257 2BC0             SUB      AX,AX
&0060:3259 EBB8             JMP      3213

 *===========================================================================*/
PRIVATE int recalibrate(fp)
register struct floppy *fp;	/* pointer tot he drive struct */
{
/* The floppy disk controller has no way of determining its absolute arm
 * position (cylinder).  Instead, it steps the arm a cylinder at a time and
 * keeps track of where it thinks it is (in software).  However, after a
 * SEEK, the hardware reads information from the diskette telling where the
 * arm actually is.  If the arm is in the wrong place, a recalibration is done,
 * which forces the arm to cylinder 0.  This way the controller can get back
 * into sync with reality.
 */

  int r;
  /* Issue the RECALIBRATE command and wait for the interrupt. */
  start_motor(fp);		/* can't recalibrate with motor off */
  fdc_out(FDC_RECALIBRATE);	/* tell drive to recalibrate itself */
  fdc_out(fp->fl_drive);	/* specify drive */
  if (need_reset) return(ERR_SEEK);	/* don't wait if controller is sick */
  receive(HARDWARE, &mess);	/* wait for interrupt message */

  /* Determine if the recalibration succeeded. */
  fdc_out(FDC_SENSE);		/* issue SENSE command to see where we are */
  r = fdc_results(fp);		/* get results of the SENSE command */
  fp->fl_curcyl = -1;		/* force a SEEK next time */
  if (r != OK ||		/* controller would not respond */
     (fp->fl_results[ST0]&ST0_BITS) != SEEK_ST0 || fp->fl_results[ST_PCN] !=0){
	/* Recalibration failed.  FDC must be reset. */
	need_reset = TRUE;
	fp->fl_calibration = UNCALIBRATED;
	return(ERR_RECALIBRATE);
  } else {
	/* Recalibration succeeded. */
	fp->fl_calibration = CALIBRATED;
	return(OK);
  }
}


/*===========================================================================*
 *				reset					     *

&0060:325B B80600           MOV      AX,0006
&0060:325E E83710           CALL     4298               ; goto enter_proc
&0060:3261 C70666560000     MOV      [5666],0000
&0060:3267 E8F00F           CALL     425A
&0060:326A C70660560000     MOV      [5660],0000
&0060:3270 C70662560000     MOV      [5662],0000
&0060:3276 2BC0             SUB      AX,AX
&0060:3278 50               PUSH     AX
&0060:3279 B8F203           MOV      AX,03F2
&0060:327C 50               PUSH     AX
&0060:327D E88C0F           CALL     420C
&0060:3280 83C404           ADD      SP,0004
&0060:3283 B80C00           MOV      AX,000C
&0060:3286 50               PUSH     AX
&0060:3287 B8F203           MOV      AX,03F2
&0060:328A 50               PUSH     AX
&0060:328B E87E0F           CALL     420C
&0060:328E 83C404           ADD      SP,0004
&0060:3291 E8CD0F           CALL     4261
&0060:3294 B86C56           MOV      AX,566C
&0060:3297 50               PUSH     AX
&0060:3298 B8FFFF           MOV      AX,FFFF
&0060:329B 50               PUSH     AX
&0060:329C E8D314           CALL     4772
&0060:329F 83C404           ADD      SP,0004
&0060:32A2 BF2856           MOV      DI,5628
&0060:32A5 C6451200         MOV      [DI+12],00
&0060:32A9 B80800           MOV      AX,0008
&0060:32AC 50               PUSH     AX
&0060:32AD E8D8FE           CALL     3188
&0060:32B0 83C402           ADD      SP,0002
&0060:32B3 57               PUSH     DI
&0060:32B4 E847FE           CALL     30FE
&0060:32B7 83C402           ADD      SP,0002
&0060:32BA 8946F8           MOV      [BP-08],AX
&0060:32BD 8A4512           MOV      AL,[DI+12]
&0060:32C0 98               CBW
&0060:32C1 2AE4             SUB      AH,AH
&0060:32C3 8946F6           MOV      [BP-0A],AX
&0060:32C6 B80300           MOV      AX,0003
&0060:32C9 50               PUSH     AX
&0060:32CA E8BBFE           CALL     3188
&0060:32CD 83C402           ADD      SP,0002
&0060:32D0 B8DF00           MOV      AX,00DF
&0060:32D3 50               PUSH     AX
&0060:32D4 E8B1FE           CALL     3188
&0060:32D7 83C402           ADD      SP,0002
&0060:32DA B80200           MOV      AX,0002
&0060:32DD 50               PUSH     AX
&0060:32DE E8A7FE           CALL     3188
&0060:32E1 83C402           ADD      SP,0002
&0060:32E4 C746FA0000       MOV      [BP-06],0000
&0060:32E9 EB12             JMP      32FD
&0060:32EB 8B46FA           MOV      AX,[BP-06]
&0060:32EE BA1C00           MOV      DX,001C
&0060:32F1 F7EA             IMUL     DX
&0060:32F3 8BD8             MOV      BX,AX
&0060:32F5 C687425600       MOV      [BX+5642],00
&0060:32FA FF46FA           INC      WORD [BP-06]
&0060:32FD 837EFA02         CMP      [BP-06],0002
&0060:3301 7CE8             JL       32EB
&0060:3303 E9C00F           JMP      42C6               ; goto leave_proc

 *===========================================================================*/
PRIVATE reset()
{
/* Issue a reset to the controller.  This is done after any catastrophe,
 * like the controller refusing to respond.
 */

  int i, r, status;
  register struct floppy *fp;
  /* Disable interrupts and strobe reset bit low. */
  need_reset = FALSE;
  lock();
  motor_status = 0;
  motor_goal = 0;
  port_out(DOR, 0);		/* strobe reset bit low */
  port_out(DOR, ENABLE_INT);	/* strobe it high again */
  unlock();			/* interrupts allowed again */
  receive(HARDWARE, &mess);	/* collect the RESET interrupt */

  /* Interrupt from the reset has been received.  Continue resetting. */
  fp = &floppy[0];		/* use floppy[0] for scratch */
  fp->fl_results[0] = 0;	/* this byte will be checked shortly */
  fdc_out(FDC_SENSE);		/* did it work? */
  r = fdc_results(fp);		/* get results */
  status = fp->fl_results[0] & BYTE;

  /* Tell FDC drive parameters. */
  fdc_out(FDC_SPECIFY);		/* specify some timing parameters */
  fdc_out(SPEC1);		/* step-rate and head-unload-time */
  fdc_out(SPEC2);		/* head-load-time and non-dma */

  for (i = 0; i < NR_DRIVES; i++) floppy[i].fl_calibration = UNCALIBRATED;
}


/*===========================================================================*
 *				clock_mess				     *
 *===========================================================================*/
PRIVATE clock_mess(ticks, func)
int ticks;			/* how many clock ticks to wait */
int (*func)();			/* function to call upon time out */
{
/* Send the clock task a message. */

  mess.m_type = SET_ALARM;
  mess.CLOCK_PROC_NR = FLOPPY;
  mess.DELTA_TICKS = ticks;
  mess.FUNC_TO_CALL = func;
  sendrec(CLOCK, &mess);
}


/*===========================================================================*
 *				send_mess				     *
 *===========================================================================*/
PRIVATE send_mess()
{
/* This routine is called when the clock task has timed out on motor startup.*/

  mess.m_type = MOTOR_RUNNING;
  send(FLOPPY, &mess);
}
