#line 1 "C:/data/emcfab/applications/firmware/extruderControler/eeprom.c"
#line 1 "c:/data/emcfab/applications/firmware/extrudercontroler/eeprom.h"
typedef unsigned char byte;
typedef unsigned int word;


void Eeprom_Write_Obj(word addr,void *obj,byte size);
void Eeprom_Read_Obj(word addr,void *obj,byte size);
#line 3 "C:/data/emcfab/applications/firmware/extruderControler/eeprom.c"
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
