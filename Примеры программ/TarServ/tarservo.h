#ifndef TARSERVOH
#define TARSERVOH

#define ToGrad 57.2957795130823
#define M_PI   3.14159265358979323846
#define D_PI 	6.28318530717958647692
#define M_PI_2 1,57079632679489661

// Constants that define available card sizes, 8MB through 128MB                                       
#define PS_8MB       8388608L
#define PS_16MB      16777216L
#define PS_32MB      33554432L
#define PS_64MB      67108864L
#define PS_128MB     134217728L

// Physical size in bytes of one MMC FLASH sector
#define PHYSICAL_BLOCK_SIZE     512    

// Erase group size = 16 MMC FLASH sectors
#define PHYSICAL_GROUP_SIZE     PHYSICAL_BLOCK_SIZE * 16

// Log table start address in MMC FLASH
#define LOG_ADDR     0x00000000

// Size in bytes for each log entry
#define LOG_ENTRY_SIZE sizeof(LOG_ENTRY)

#define BUFFER_ENTRIES 32

// Size of XRAM memory buffer that stores table entries
// before they are written to MMC
#define BUFFER_SIZE  LOG_ENTRY_SIZE * BUFFER_ENTRIES

// Command table value definitions
// Used in the MMC_Command_Exec function to 
// decode and execute MMC command requests
#define     EMPTY  0
#define     YES   1
#define     NO    0
#define     CMD   0
#define     RD    1
#define     WR    2
#define     R1    0
#define     R1b   1
#define     R2    2
#define     R3    3

// Start and stop data tokens for single and multiple
// block MMC data operations
#define     START_SBR      0xFE
#define     START_MBR      0xFE
#define     START_SBW      0xFE
#define     START_MBW      0xFC
#define     STOP_MBW       0xFD

// Mask for data response token after an MMC write
#define     DATA_RESP_MASK 0x11

// Mask for busy token in R1b response
#define     BUSY_BIT       0x80

// Command Table Index Constants:
// Definitions for each table entry in the command table.
// These allow the MMC_Command_Exec function to be called with a
// meaningful parameter rather than a number.
#define     GO_IDLE_STATE            0
#define     SEND_OP_COND             1
#define     SEND_CSD                 2
#define     SEND_CID                 3
#define     STOP_TRANSMISSION        4
#define     SEND_STATUS              5
#define     SET_BLOCKLEN             6
#define     READ_SINGLE_BLOCK        7
#define     READ_MULTIPLE_BLOCK      8
#define     WRITE_BLOCK              9
#define     WRITE_MULTIPLE_BLOCK    10
#define     PROGRAM_CSD             11
#define     SET_WRITE_PROT          12
#define     CLR_WRITE_PROT          13
#define     SEND_WRITE_PROT         14
#define     TAG_SECTOR_START        15
#define     TAG_SECTOR_END          16
#define     UNTAG_SECTOR            17
#define     TAG_ERASE_GROUP_START   18
#define     TAG_ERASE_GROUP_END     19
#define     UNTAG_ERASE_GROUP       20
#define     ERASE                   21
#define     LOCK_UNLOCK             22
#define     READ_OCR                23
#define     CRC_ON_OFF              24


//-----------------------------------------------------------------------------
// UNIONs, STRUCTUREs, and ENUMs
//-----------------------------------------------------------------------------
typedef union LONG {                   // byte-addressable LONG
   long l;
   unsigned char b[4];
} LONG;

typedef union INT {                    // byte-addressable INT
   int i;
   unsigned char b[2];
} INT;

typedef union {                        // byte-addressable unsigned long
      unsigned long l;
      unsigned char b[4];
              } ULONG;

typedef union {                        // byte-addressable unsigned int
      unsigned int i;
      unsigned char b[2];
              } UINT;

typedef struct LOG_ENTRY {             // (7 bytes per entry)
   int wTemp;                          // temperature in hundredths of a
                                       // degree
   unsigned int uDay;                  // day of entry
   unsigned char bHour;                // hour of entry
   unsigned char bMin;                 // minute of entry
   unsigned char bSec;                 // second of entry
   unsigned char pad;                  // dummy byte to ensure aligned access;
} LOG_ENTRY;
     
// The states listed below represent various phases of 
// operation;
typedef enum STATE {
   RESET,                              // Device reset has occurred;
   RUNNING,                            // Data is being logged normally;
   FINISHED,                           // Logging stopped, store buffer;
   STOPPED                             // Logging completed, buffer stored;
   } STATE;

// This structure defines entries into the command table;
typedef struct {
      unsigned char command_byte;      // OpCode;
      unsigned char arg_required;      // Indicates argument requirement;
      unsigned char CRC;               // Holds CRC for command if necessary;
      unsigned char trans_type;        // Indicates command transfer type;
      unsigned char response;          // Indicates expected response;
      unsigned char var_length;        // Indicates varialble length transfer;
               } COMMAND;

// Command table for MMC.  This table contains all commands available in SPI
// mode;  Format of command entries is described above in command structure
// definition;
COMMAND code commandlist[25] = {
      { 0,NO ,0x95,CMD,R1 ,NO },    // CMD0;  GO_IDLE_STATE: reset card;
      { 1,NO ,0xFF,CMD,R1 ,NO },    // CMD1;  SEND_OP_COND: initialize card;
      { 9,NO ,0xFF,RD ,R1 ,NO },    // CMD9;  SEND_CSD: get card specific data;
      {10,NO ,0xFF,RD ,R1 ,NO },    // CMD10; SEND_CID: get card identifier;
      {12,NO ,0xFF,CMD,R1 ,NO },    // CMD12; STOP_TRANSMISSION: end read;
      {13,NO ,0xFF,CMD,R2 ,NO },    // CMD13; SEND_STATUS: read card status;
      {16,YES,0xFF,CMD,R1 ,NO },    // CMD16; SET_BLOCKLEN: set block size;
      {17,YES,0xFF,RD ,R1 ,NO },    // CMD17; READ_SINGLE_BLOCK: read 1 block;
      {18,YES,0xFF,RD ,R1 ,YES},    // CMD18; READ_MULTIPLE_BLOCK: read > 1;
      {24,YES,0xFF,WR ,R1 ,NO },    // CMD24; WRITE_BLOCK: write 1 block;
      {25,YES,0xFF,WR ,R1 ,YES},    // CMD25; WRITE_MULTIPLE_BLOCK: write > 1;
      {27,NO ,0xFF,CMD,R1 ,NO },    // CMD27; PROGRAM_CSD: program CSD;
      {28,YES,0xFF,CMD,R1b,NO },    // CMD28; SET_WRITE_PROT: set wp for group;
      {29,YES,0xFF,CMD,R1b,NO },    // CMD29; CLR_WRITE_PROT: clear group wp;
      {30,YES,0xFF,CMD,R1 ,NO },    // CMD30; SEND_WRITE_PROT: check wp status;
      {32,YES,0xFF,CMD,R1 ,NO },    // CMD32; TAG_SECTOR_START: tag 1st erase;
      {33,YES,0xFF,CMD,R1 ,NO },    // CMD33; TAG_SECTOR_END: tag end(single);
      {34,YES,0xFF,CMD,R1 ,NO },    // CMD34; UNTAG_SECTOR: deselect for erase;
      {35,YES,0xFF,CMD,R1 ,NO },    // CMD35; TAG_ERASE_GROUP_START;
      {36,YES,0xFF,CMD,R1 ,NO },    // CMD36; TAG_ERASE_GROUP_END;
      {37,YES,0xFF,CMD,R1 ,NO },    // CMD37; UNTAG_ERASE_GROUP;
      {38,YES,0xFF,CMD,R1b,NO },    // CMD38; ERASE: erase all tagged sectors;
      {42,YES,0xFF,CMD,R1b,NO },    // CMD42; LOCK_UNLOCK;
      {58,NO ,0xFF,CMD,R3 ,NO },    // CMD58; READ_OCR: read OCR register;
      {59,YES,0xFF,CMD,R1 ,NO }    // CMD59; CRC_ON_OFF: toggles CRC checking;
                              };

//-----------------------------------------------------------------------------
// Global VARIABLES
//-----------------------------------------------------------------------------

xdata LONG Result = {0L};              // ADC0 decimated value

xdata LOG_ENTRY LogRecord;             // Memory space for each log entry
xdata unsigned long uLogCount;         // Current number of table entries
LOG_ENTRY xdata *pLogTable;            // Pointer to buffer for table entries
xdata STATE State = RESET;             // System state variable;  Determines
                                       // how log update function will exec;
xdata unsigned long PHYSICAL_SIZE;     // MMC size variable;  Set during
                                       // initialization;
xdata unsigned long LOG_SIZE;          // Available number of bytes for log
                                       // table;

xdata unsigned long PHYSICAL_BLOCKS;   // MMC block number;  Computed during
                                       // initialization;

xdata char LOCAL_BLOCK[BUFFER_SIZE]; 
xdata char SCRATCH_BLOCK[PHYSICAL_BLOCK_SIZE];

xdata char error;

// High Level MMC_FLASH Functions

void MMC_FLASH_Init (void);            // Initializes MMC and configures it to 
                                       // accept SPI commands;
                                       // Reads <length> bytes starting at 
                                       // <address> and stores them at <pchar>;
unsigned char MMC_FLASH_Read (unsigned long address, unsigned char *pchar,
                         unsigned int length);

                                       // Clears <length> bytes starting at 
                                       // <address>; uses memory at <scratch>
                                       // for temporary storage;
unsigned char MMC_FLASH_Clear (unsigned long address, unsigned char *scratch,
                          unsigned int length);

                                       // Writes <length> bytes of data at
                                       // <wdata> to <address> in MMC;
                                       // <scratch> provides temporary storage;
unsigned char MMC_FLASH_Write (unsigned long address, unsigned char *scratch,
                          unsigned char *wdata, unsigned int length);

                                       // Clears <length> bytes of FLASH 
                                       // starting at <address1>; Requires that
                                       // desired erase area be sector aligned;
unsigned char MMC_FLASH_MassErase (unsigned long address1, 
                                   unsigned long length);

// Low Level MMC_FLASH_ Functions

                                       // Decodes and executes MMC commands;  
                                       // <cmd> is an index into the command
                                       // table and <argument> contains a 
                                       // 32-bit argument if necessary;  If a 
                                       // data operation is taking place, the
                                       // data will be stored to or read from
                                       // the location pointed to by <pchar>;
unsigned int MMC_Command_Exec (unsigned char cmd, unsigned long argument,
                           unsigned char *pchar);




#endif