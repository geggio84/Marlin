#ifndef CARDREADER_H
#define CARDREADER_H

#include "Marlin.h"

#ifdef SDSUPPORT

#define MAX_DIR_DEPTH 10
/** Number of VFAT entries used. Every entry has 13 UTF-16 characters */
#define MAX_VFAT_ENTRIES (2)
/** Total size of the buffer used to store the long filenames */
#define LONG_FILENAME_LENGTH (13*MAX_VFAT_ENTRIES+1)

//#include "SdFile.h"
enum LsAction {LS_SerialPrint,LS_Count,LS_GetFilename};
class CardReader
{
public:
  CardReader();
  
  void initsd();
  void write_command(char *buf);
  //files auto[0-9].g on the sd card are performed in a row
  //this is to delay autostart and hence the initialisaiton of the sd card to some seconds after the normal init, so the device is available quick after a reset

  void checkautostart(bool x); 
  void openFile(char* name,bool read,bool replace_current=true);
  void openLogFile(char* name);
  void removeFile(char* name);
  void closefile(bool store_location=false);
  void release();
  void startFileprint();
  void pauseSDPrint();
  void getStatus();
  void printingHasFinished(myFILE *gcode_file);

  void getfilename(const uint8_t nr);
  uint16_t getnrfilenames();
  
  void getAbsFilename(char *t);
  

  void ls();
  void chdir(const char * relpath);
  void updir();
  void setroot();

  FORCE_INLINE bool isFileOpen() { return false; }
  FORCE_INLINE bool eof() { return sdpos>=filesize ;};
  //FORCE_INLINE int16_t get() {  sdpos = file.curPosition();return (int16_t)file.read();};
  FORCE_INLINE int16_t get(myFILE *file) {  sdpos = file->currpos; file->currpos++; return (int16_t)fgetc(file->file_p);};
  FORCE_INLINE void setIndex(long index) {sdpos = index;};//file.seekSet(index);};
  FORCE_INLINE uint8_t percentDone(){if(!isFileOpen()) return 0; if(filesize) return sdpos/((filesize+99)/100); else return 0;};
  //FORCE_INLINE char* getWorkDirName(){workDir.getFilename(filename);return filename;};

public:
  bool saving;
  bool logging;
  bool sdprinting ;  
  bool cardOK ;
  char filename[13];
  char longFilename[LONG_FILENAME_LENGTH];
  bool filenameIsDir;
  int lastnr; //last number of the autostart;
private:
/* TODO: FIXME */
  //SdFile root,*curDir,workDir,workDirParents[MAX_DIR_DEPTH];
/* TODO: FIXME */
  uint16_t workDirDepth;
/* TODO: FIXME */
  //Sd2Card card;
  //SdVolume volume;
  //SdFile file;
/* TODO: FIXME */
  #define SD_PROCEDURE_DEPTH 1
  #define MAXPATHNAMELENGTH (13*MAX_DIR_DEPTH+MAX_DIR_DEPTH+1)
  uint8_t file_subcall_ctr;
  uint32_t filespos[SD_PROCEDURE_DEPTH];
  char filenames[SD_PROCEDURE_DEPTH][MAXPATHNAMELENGTH];
  uint32_t filesize;
  //int16_t n;
  unsigned long autostart_atmillis;
  uint32_t sdpos ;

  bool autostart_stilltocheck; //the sd start is delayed, because otherwise the serial cannot answer fast enought to make contact with the hostsoftware.
  
  LsAction lsAction; //stored for recursion.
  int16_t nrFiles; //counter for the files in the current directory and recycled as position counter for getting the nrFiles'th name in the directory.
  char* diveDirName;
/* TODO: FIXME */
  //void lsDive(const char *prepend,SdFile parent);
/* TODO: FIXME */
};
extern CardReader card;
#define IS_SD_PRINTING (card.sdprinting)

/* TODO: FIXME */
/*#if (SDCARDDETECT > -1)
# ifdef SDCARDDETECTINVERTED 
#  define IS_SD_INSERTED (READ(SDCARDDETECT)!=0)
# else
#  define IS_SD_INSERTED (READ(SDCARDDETECT)==0)
# endif //SDCARDTETECTINVERTED
#else
//If we don't have a card detect line, aways asume the card is inserted
# define IS_SD_INSERTED true
#endif*/
/* TODO: FIXME */

#else

#define IS_SD_PRINTING (false)

#endif //SDSUPPORT
#endif
