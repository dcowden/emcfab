#include "eeprom.h"

void Eeprom_Write_Obj(word addr,void *obj,byte size)
{
      byte i,*ptr=(byte *)obj;
      for (i=0;i<size;i++)
            Eeprom_Write(addr++,*(ptr++));
}

void Eeprom_Read_Obj(word addr,void *obj,byte size)
{
      byte i,*ptr=obj;
      for (i=0;i<size;i++)
            *(ptr++)=Eeprom_Read(addr++);
}

